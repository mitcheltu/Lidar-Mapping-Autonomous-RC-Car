# On-Demand LiDAR: Sensor Modes + iOS App Redesign

**Date:** 2026-08-03
**Status:** approved, implementing

## Problem

The phone streams depth continuously from the moment it connects, and holds the
entire cloud on-device in a voxel dictionary capped at 400,000 cells. Three things
are wrong with that for a car:

1. **The cap is a silent kill switch.** In `PointCloudAccumulator.add()`, the
   `pendingNew.append(...)` that feeds streaming lives inside the same
   `else if cells.count < maxPoints` branch that creates a cell. Once the cap is
   reached, no new cells are created, so `pendingNew` stays empty and
   `drainNew()` returns nothing **forever**. The phone looks connected and sends
   no geometry.
2. **The on-device dedup is redundant.** The laptop already runs a log-odds
   `VoxelGrid` with ray carving. The phone is duplicating that work, and paying
   for it with the cap and with memory/thermal load.
3. **Depth runs when nobody wants it.** Unprojecting every third frame costs
   battery and heat while the car is parked, planning, or waiting.

Streaming only *new* voxels also starves the laptop's log-odds grid: a surface
observed a hundred times is sent once, so it never accrues the repeated evidence
the grid needs to hold it occupied against carving.

## Design

**The phone becomes a stateless, mode-driven sensor.** Unproject → subsample →
send. No accumulator, no cap, no `drainNew()`.

### Modes

The laptop owns the decision and pushes it over the existing reverse channel.

| Mode | Pose | Depth | When |
|---|---|---|---|
| `IDLE` | 10 Hz | off | parked, planning, waiting — the default on connect |
| `SCAN` | 10 Hz | full rate | during a commanded stationary 360° spin |
| `DRIVE` | 10 Hz | full rate | while the car is moving |

`SCAN` and `DRIVE` both process depth; they stay distinct so the rates can
diverge later without another protocol change. The win is `IDLE`, which is where
the car spends most of its time.

### Per-frame budget (replaces the accumulator cap)

Dropping dedup means each processed frame carries its whole depth sample. At
`pixelStride=2` that is ~12k points/frame, ~10 fps, ~1.8 MB/s — and
`voxel_mapper_node` currently needs ~2 s per cloud. So each frame is subsampled to
a fixed budget of **2000 points** (matching the mapper's `max_rays`), chosen by a
stride computed per frame. Bounded bandwidth, bounded mapper cost, and no state
retained between frames.

This is the important distinction: no cap on *accumulation* (there is no
accumulation), but a hard bound per *frame*.

### Protocol

One new message, laptop → phone, in the existing framing
(`type(1) + len(uint32 LE) + payload`):

```
'M' 0x4D  mode : ASCII "IDLE" | "SCAN" | "DRIVE"
```

`bridge_node` already relays `/drive` as `0x44 'D'`; the phone no longer needs it
(driving goes laptop → ESP32 directly over WiFi), and ignores unknown types. The
change is therefore **backward compatible**: an un-updated app keeps behaving
exactly as it does today.

### Scan orchestration

The laptop drives the spin; the phone never decides to move.

Pressing `s` in the console publishes `/scan_trigger`, and `scan_node` runs:

1. stop the car, wait for it to settle
2. `MODE SCAN`
3. rotate in place at `config.SPIN_SPEED`, accumulating |Δθ| from `/pose` until
   360° (or a timeout)
4. stop
5. `MODE IDLE`

Motor ownership reuses the arbitration built for calibration: `scan_node`
publishes a latched `/scan_active` and drives through `/drive_raw`, which
`car_driver_node` obeys instead of `/drive` while either scan or calibration is
active. Exactly one owner of the motors at any instant.

Aborting: `h`/SPACE stops the scan as it stops everything else.

### iOS changes

- **`PointCloudStreamer`** — add a receive loop that parses inbound frames and
  publishes the current mode; add a raw `sendPoints(positions:colors:count:)`
  that takes a flat buffer rather than `[PointVertex]`.
- **`ARDepthView.Coordinator`** — skip all depth work unless the mode wants it;
  unproject straight into a reusable flat buffer with a per-frame stride; send
  immediately.
- **`PointCloudAccumulator` and `PLYExporter`** — deleted. The phone is bolted to
  a car; the map is visible in Rerun.
- **`ContentView`** — show the mode and a streamed-points-per-second figure
  instead of an accumulated point count; drop the PLY/scan-toggle UI.

## Verification

- `nav.stream_protocol` mode encode/decode: pytest, TDD, hardware-free.
- `scan_node`: exercised in WSL against synthetic `/pose` rotation — the spin
  terminates at 360°, emits the mode sequence in order, and leaves
  `/scan_active` false on every exit path including abort.
- `bridge_node` mode relay: verified against a stub socket client.
- **The Swift cannot be built or tested in this environment** (no macOS/Xcode).
  It is written to compile against the existing app structure; the user builds
  and runs it. Everything on the laptop side is written to degrade gracefully
  against the un-updated app.

## Out of scope

- Obstacle tripwire depth while driving (rejected: full depth kept while driving).
- Automatic scan triggers on arrival or map staleness (console key only for now).
- The mapper's throughput problem, tracked separately.
