# Autonomous RC Car — Project Status (Single Source of Truth)

> **This file is the hub.** It condenses the scattered design docs into one place
> and points to the detailed ones. When status changes, update *this* file and
> `TODO.md`. Last consolidated: **2026-07-27**.

## 1. What this project is

A small floor robot whose "brain" is an **iPhone 12 Pro Max** mounted on top. The
phone's ARKit LiDAR builds a 3D point cloud + 6-DoF pose of the room, decides
where to drive, and sends `L<left>R<right>` motor commands to an **ESP32** over
**BLE**. The ESP32 drives two logical sides (differential/tank drive) through an
H-bridge. A **laptop** receives the live stream over Wi-Fi for visualization and
runs the Python navigation prototype.

Proven-architecture reference: **RoBart** (iPhone-brain + BLE microcontroller).

## 2. Current state (what actually runs today)

| Piece | State |
|---|---|
| iPhone LiDAR scanner + live Wi-Fi stream | **Working** (perception + PLY export + pose/points/JPEG stream) |
| Laptop Python `nav/` package (Milestone A) | **Working**, unit-tested (grid, mapping, localization, frontier, planner, overlay, preview) |
| `pc_viewer.py` walkthrough preview (`G` key) | **Working** — map the room with just the phone, no car |
| Car hardware (chassis/motors/H-bridge) | **Not built** — parts not purchased; ESP32 owned but unwired |
| ESP32 firmware (`esp32_car.ino`) | Written, **not flashed/field-tested** |
| ROS2 ament package (`autonomous_rc_car_ros`) | **Real** — builds/runs in WSL2. All 5 nodes implemented: `bridge_node`, `voxel_mapper_node` (incremental log-odds voxels + ray carving), `frontier_planner_node` (A* path), `motion_controller_node` (drive cmds), `icp_slam_node` (ICP drift correction). Unified Z-up `map` frame. |
| Autonomy driving loop (Milestone B) | **Software complete** — full perception→SLAM→plan→drive chain wired and unit-tested. Remaining is hardware: build the car, flash/tune the ESP32, field test. |

**Bottom line:** the only end-to-end runnable path is **phone + laptop** (scan a
room, watch/preview the map). Nothing drives yet.

## 3. Architecture — **canonical: ROS2 + Nav2 on the laptop**

**Decision (2026-07-27):** adopt the ROS2 + Nav2 structure as canonical (per the
`autonomous_rc_car/README.md` Technical Spec). The laptop runs a real ROS2 graph;
the phone is a sensor+actuator bridge; the ESP32 drives the motors.

```
iPhone (ARKit LiDAR + 6-DoF pose)
   │  Wi-Fi stream (points / pose / JPEG)  +  reverse channel (DRIVE / MODE)
   ▼
Laptop — ROS2 graph
   bridge_node ──► /points /pose /image        (ingest the iPhone stream)
   icp_slam_node ──► /pose_corrected           (KISS-ICP scan-to-map drift fix)
   voxel_mapper_node ──► /map (OccupancyGrid)   (height-banded grid + inflation)
   Nav2 (or frontier_planner_node + planner) ──► /cmd path/waypoints
   motion_controller ──► /drive  (L<left>R<right>)
   viewer (pc_viewer.py / PyVista) ── live 3D + overlays
   │  DRIVE command back over the reverse channel
   ▼
iPhone relays via BLE  ──►  ESP32  ──►  H-bridge (TB6612 ×2)  ──►  4 motors
```

- **Differential drive**: both sides forward = straight; opposite = spin in place
  (used for the 360° scan). Firmware only ever sees two logical sides.
- **Localization**: ARKit VIO pose, corrected by KISS-ICP scan-to-map after each
  scan.
- **The proven Python `nav/` package is the algorithm library** the ROS2 nodes
  wrap — it is NOT thrown away. Nav2 handles costmap/planner/controller plumbing;
  the custom `nav/` frontier + A* logic remains available where Nav2 is overkill.

> ⚠️ **Reality check (verified 2026-07-27):** the ROS2 layer is currently ROS2
> *in name only* — see §7.2. Reaching the diagram above is the Milestone B/C work
> in `TODO.md`.

**Fallback if full ROS2/Nav2 proves too heavy:** the grounded plain-Python path
(the `nav/` package driven by `pc_viewer.py` + a socket reverse channel, exactly
as the 19-task plan describes) still works and can drive the car without ROS2.
Keep it as the escape hatch.

## 4. Repository layout (current, actual)

```
Remote Car/
├── PROJECT_STATUS.md          # ← you are here (the hub)
├── TODO.md                    # action tracker (completed / remaining)
├── Build-Plan.md              # full build narrative + BOM + roadmap
├── Electronics-Shopping-List.md  # 4WD parts list + wiring rules
├── Navigation-Pipeline.md     # point-cloud → navigable-map algorithm design
├── docs/superpowers/plans/2026-07-12-lidar-navigation-autonomy.md  # 19-task TDD plan
└── autonomous_rc_car/
    ├── README.md              # "Technical Spec" (ROS2/Nav2/KISS-ICP/PyVista vision — see §7)
    ├── ROS2_SETUP.md          # WSL2 + ROS2 Humble install / build / run guide
    ├── ros2_ws/src/autonomous_rc_car_ros/  # ★ thin ament package: rclpy nodes that import nav.*
    │                          #   (bridge_node real; voxel_mapper/frontier_planner/
    │                          #    motion_controller/icp_slam nodes are stubs)
    ├── laptop_brain/          # Python: nav LIBRARY + viewer + tests (pip-installable)
    │   ├── pyproject.toml      # exposes `nav` as the `rc-car-nav` library
    │   ├── nav/               # ★ ALGORITHM LIBRARY: grid, mapping, localization, frontier,
    │   │                      #   planner, overlay, preview, voxel_viewer, stream_protocol
    │   ├── nodes/             # pre-ROS2 CLI scripts + stream_protocol shim (being superseded)
    │   ├── pc_viewer.py       # ★ CANONICAL viewer: live Open3D + pose marker + camera + G-preview
    │   ├── tests/             # pytest suite (hardware-free, 57 passing)
    │   └── captures/          # recorded point-cloud sessions (~216 MB — gitignored)
    ├── esp32_firmware/        # esp32_car.ino, tb6612 variant, platformio project
    └── ios_app/RCCarLidarStreamer/  # SwiftUI + ARKit app (see naming note in §8)
```

**Viewer consolidation (done):** there were two viewers — the full-featured
`pc_viewer.py` (pose marker, camera feed, disk recording, `G`-key preview) and a
stripped-down `nodes/live_viewer.py` that only drew points. The redundant
`live_viewer.py` was **removed**; `pc_viewer.py` is the single canonical viewer.
The shared wire protocol lives in `nodes/stream_protocol.py` (tested); the plan is
to extend it to cover pose/JPEG and have `pc_viewer.py` use it (one protocol,
one viewer).

## 5. The navigation pipeline (condensed)

Runs on the accumulated map every ~1–2 s (details in `Navigation-Pipeline.md`):

1. **Downsample** — voxel grid, 3–5 cm cells.
2. **Remove outliers** — statistical + radius removal (kills flying specks).
3. **Split floor vs obstacles** — gravity-aligned height bands (floor / robot-height
   obstacle / ignore-overhead).
4. **Occupancy grid + inflation** — 5 cm cells, EDT inflation by robot radius (~0.12 m).
5. **Localize** — ARKit pose → grid cell `(x, z, θ)`.
6. **Pick next target** — Yamauchi frontier detection + BFS reachability +
   nearest-frontier goal; **A\*** path (8-connected, no corner cutting) + LOS
   simplification.

Then: drive waypoints closed-loop → 360° rescan → repeat until no reachable
frontier remains.

## 6. Milestones & status

The 19-task plan (`docs/superpowers/plans/...`) splits into two milestones:

- **Milestone A — phone-only walkthrough mapping (DONE):** Tasks 1–8, 15, 15b.
  The `nav/` package + `G`-key preview. Reviewed; ~46 tests passing at last run.
- **Milestone B — car autonomy (REMAINING):** Tasks 9–14, 16–19 — waypoint
  controller, explorer state machine, ICP drift, PC↔phone command protocol,
  NavRunner failsafes, `pc_viewer` integration, Swift bidirectional streamer +
  scan gating, and the field-test/calibration checklist.

**Two performance follow-ups** flagged before Milestone B (Tasks 10/16):
- `bfs_distances` worst case ~4.7 s on open 400×400 grids — vectorize/offload.
- `simplify_path` has a super-quadratic maze worst case.
- Preview hitches the GUI thread every ~2 s on large clouds — move off-thread.

## 7. Decisions (resolved 2026-07-27) & the ROS2 reality check

**Resolved by the user:**
- **7.1 — Git baseline.** Reconcile the structure, then **treat the current tree
  as the start of the git repo** (fresh initial commit). The old `viewer/`,
  `firmware/`, `PointCloudScanner/` history is not carried forward. *(Blocked on
  freeing disk — see 7.4.)*
- **7.2 — Architecture.** **ROS2 + Nav2 is canonical** (§3). The grounded
  plain-Python path is kept only as a fallback.
- **Viewers.** `pc_viewer.py` is canonical; redundant `live_viewer.py` removed (§4).

**7.2 verification — does the ROS2 structure make sense today? Not yet.** I checked:
- **No node imports `rclpy`.** The `nodes/*.py` are plain `argparse` CLI scripts
  that call into the `nav/` library — not ROS2 nodes (no publishers/subscribers,
  no spin). `motion_controller.py` is a standalone pure-pursuit class.
- **No ROS2 package exists** — there is no `package.xml`, `setup.py`, or
  `setup.cfg`, so `colcon build` / `ros2 run` can't work. The `launch/*.launch.py`
  files reference `package="autonomous_rc_car"` which isn't a real ament package.
- **The `config/*.yaml` are unused** — nothing reads `nav2_params.yaml` or
  `kiss_icp_params.yaml`.
- **Nav2 / KISS-ICP / PyVista are not installed or wired.**

**Conclusion:** the ROS2/Nav2 target is sound, but the current `nodes/`+`launch/`+
`config/` are aspirational scaffolding. Reaching it means *building a real ament
package and converting the scripts into rclpy nodes* — that is the plan in
`TODO.md` (Milestone B: ROS2 foundation). Note ROS2 on **Windows** is awkward;
consider WSL2/Ubuntu or Docker for the laptop_brain (flagged in the plan).

**Still-open cleanups (not blocking, tracked in `TODO.md`):**
- **7.3 — Stale doc paths & iOS naming.** Docs reference `viewer/`, `firmware/`,
  `PointCloudScanner/`; app is `RCCarLidarStreamer` on disk but "PointCloudScanner"
  in docs, with duplicated Swift files. Needs a find-replace + de-dup pass.
- **7.4 — Disk full (blocks the git baseline).** `C:` is at 100% (≈0 bytes free);
  blocks `git init`/commit and `pip install`. Reclaim targets: `.venv312/`
  (721 MB), `captures/` (216 MB), `.venv/` (22 MB). Not deleted — your call.

## 8. Document index (what each file is for)

| Doc | Purpose | Status |
|---|---|---|
| `PROJECT_STATUS.md` | **Hub** — current state, architecture, decisions | canonical |
| `TODO.md` | Action tracker (done / remaining) | canonical |
| `Build-Plan.md` | Origin story, full BOM tiers, autonomy roadmap, pitfalls | reference (stale paths) |
| `Electronics-Shopping-List.md` | 4WD parts + wiring rules + bring-up order | reference |
| `Navigation-Pipeline.md` | Algorithm design for map-building & frontier exploration | reference |
| `docs/superpowers/plans/2026-07-12-...md` | 19-task TDD implementation plan | reference (Milestone A done) |
| `autonomous_rc_car/README.md` | ROS2/Nav2/KISS-ICP "Technical Spec" | **canonical target** — not yet realized (§7.2) |
| `ios_app/.../README.md` | Xcode setup for the scanner app | reference (stale name/paths) |
