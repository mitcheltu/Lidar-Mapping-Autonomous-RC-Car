# Features Overview + ESP32 Connect & Tune

How the whole system works today, and exactly how to wire, flash, and tune the
ESP32 so the car drives. Companion to `PROJECT_STATUS.md`,
`Electronics-Shopping-List.md`, and `ROS2_SETUP.md`.

---

## Part 1 — What the system does today

### The pipeline (phone → laptop → car)

```
iPhone (ARKit LiDAR + VIO pose)
  │  Wi-Fi TCP stream: points / pose / jpeg          reverse: DRIVE (L..R..)
  ▼                                                        ▲
Laptop — ROS2 graph (WSL2, package autonomous_rc_car_ros)  │
  bridge_node          TCP :9000 ⇄ ROS.  points→/points, pose→/pose, jpeg→/image;
                       subscribes /drive and relays it back down the socket.
  icp_slam_node        /points+/pose → /pose_corrected  (ICP scan-to-map drift fix)
  voxel_mapper_node    /points+/pose → /map (2D costmap) + /voxels/{ground,obstacle}
                       (incremental log-odds voxel cubes with ray carving)
  frontier_planner_node/map+/pose → /cmd_path  (nearest frontier + A* + LOS simplify)
  motion_controller_node /cmd_path+/pose → /drive  (turn-then-drive waypoint follower)
  │  /drive = "L<left>R<right>", each -100..100
  ▼
iPhone relays over BLE → ESP32 → H-bridge → motors
```

All five ROS nodes are implemented and unit-tested (93 tests). Everything is
published in one **Z-up `map` frame**, viewable in RViz2 (`/points`, `/map`,
`/voxels/*`, `/pose`, `/cmd_path`).

### What works now vs. what's needed to physically drive

| Stage | Status |
|---|---|
| Phone scan → laptop stream | ✅ working |
| SLAM / map / voxels / frontier / A* path | ✅ working, live in RViz |
| `motion_controller` computing `L..R..` on `/drive` (with GO/HOLD gate) | ✅ working |
| ESP32 + TB6612 + motors | ✅ **built** |
| Laptop → ESP32 driver (`car_driver_node` → WiFi TCP) | ✅ **written** — needs flashing + a field test (Part 4) |
| Drive calibration against the phone's pose | ✅ **written** — needs a field run (Part 5) |

So the chain is complete in software from LiDAR to motor command. What remains is
physical: flash `esp32_car_wifi_tb6612.ino`, confirm the link, calibrate, drive.

---

## Part 2 — Connect the ESP32 (hardware + firmware)

Parts and full wiring rules are in `Electronics-Shopping-List.md`. **This car's
firmware is `esp32_firmware/src/esp32_car_wifi_tb6612.ino`** (TB6612 + WiFi).

### Wiring (TB6612 — matches `esp32_car_wifi_tb6612.ino`)

```
TB6612 VM   ─► motor battery + (7.4 V, 2×18650), through your switch
TB6612 VCC  ─► ESP32 3.3V        (logic reference)
TB6612 STBY ─► ESP32 GPIO 22     (must be HIGH to enable)
TB6612 GND  ─► COMMON GROUND     (battery −, ESP32 GND — one star point;
                                  the #1 thing beginners miss)

Motor A (LEFT):                    Motor B (RIGHT):
  AIN1 ─► GPIO 18                    BIN1 ─► GPIO 16
  AIN2 ─► GPIO 19                    BIN2 ─► GPIO 17
  PWMA ─► GPIO 23                    PWMB ─► GPIO 21
  AO1/AO2 ─► left motor              BO1/BO2 ─► right motor

ESP32 powered separately from its own USB battery, so motor noise stays off the
logic supply. 100 nF caps across the motors, bulk cap on VM→GND at the driver.
```

### Flash it
1. Fill in `WIFI_SSID` / `WIFI_PASS` at the top of the sketch.
2. **Arduino IDE:** Boards Manager → install "esp32". Open
   `esp32_car_wifi_tb6612.ino`, select your board, Upload.
   **PlatformIO:** `esp32_firmware/platformio.ini` is set up; `pio run -t upload`.
3. Serial Monitor @ 115200 should print the car's IP and
   `*** Listening on port 9001`.

> The older `esp32_car.ino` (L298N) and `esp32_car_tb6612.ino` are the **BLE**
> sketches on different pins. They do not match this car — keep them only as
> reference for the BLE path.

---

## Part 3 — Smoke-test and tune

### Bench test FIRST (wheels off the ground)
Prop the chassis so the wheels spin free. Prove the link + motors before autonomy —
from any terminal on the same WiFi, using the IP the sketch printed:

```bash
printf 'L60R60\n'  | nc 192.168.1.xx 9001    # both sides forward
printf 'L0R0\n'    | nc 192.168.1.xx 9001    # stop
printf 'L60R-60\n' | nc 192.168.1.xx 9001    # spin in place
```

The firmware has a **0.5 s failsafe**: no command for 500 ms and it stops. So a
single `nc` command runs for only half a second — that is correct behaviour, not a
fault. `motion_controller_node` publishes at 10 Hz, which keeps it fed.

### Motor direction
If a wheel spins the wrong way, either swap that motor's two OUT wires or flip
`LEFT_DIR` / `RIGHT_DIR` in the sketch. **Fix this before calibrating** — a
backwards side makes every measurement meaningless.

### Tuning knobs

**In the ESP32 firmware (`esp32_car_wifi_tb6612.ino`):**
- `PWM_FREQ` 20 kHz (silent) — leave as is.
- `FAILSAFE_MS` (500) — how long without a command before it stops.
- `LEFT_DIR` / `RIGHT_DIR` — flip a side that runs backwards.
- Do **not** add a duty floor here for stiction. That is what `drive_deadband` in
  the calibration is for, and a floor in firmware would fight it.

**Measured, not tuned — `config/calibration.yaml`** (press `c`, see Part 5):
`turn_sign`, `drive_deadband`, `turn_deadband`, `linear_gain`, `angular_gain`,
`straightness_trim`, `command_latency`. Delete the file to go back to
uncalibrated pass-through behaviour.

**In the nav library (`laptop_brain/nav/config.py`) — the driving behavior:**
- `DRIVE_SPEED` (45) — cruise. Keep low indoors; raise for carpet.
- `TURN_SPEED` (40) — in-place rotation speed.
- `ARRIVE_DIST` (0.10 m) — how close counts as "reached a waypoint".
- `TURN_THRESHOLD` (0.44 rad ≈ 25°) — above this heading error, turn in place
  instead of driving; lower = straighter lines but more stop-and-turn.

**Live (no rebuild) via ROS params** — e.g. throttle the mapper or tune rates:
```bash
ros2 param set /voxel_mapper_node max_rays 1000
ros2 param set /motion_controller_node rate_hz 8.0
```
(`nav/config.py` constants need the `nav` lib reloaded — just relaunch, since it's
an editable install. `calibration.yaml` is the exception: it is re-read live when
calibration writes a new one.)

### Bring-up order (never all at once)
1. Wheels off ground → `nc` smoke test (`L60R60`) → confirm both directions.
2. Add motor caps + a single **star ground**; retest (motion smooth, ESP32 never resets).
3. Mount the phone facing forward; `./run.sh --car <ip>`; confirm `car: connected`.
4. Car on the floor, ~2×2 m clear → press **c** to calibrate (Part 5).
5. Press **p** to plan, then **g** to drive. **h**/SPACE stops.

---

## Part 3b — Testing mode: plan on command, display it, drive only when you say go

Designed to be easy to watch and fully under your control:
- **Planning is on-demand.** `frontier_planner_node` stores the latest `/map` + `/pose`
  and computes **one** path only when you press **p**. `/cmd_path` is **latched**, so it
  stays put in RViz instead of flickering (add a **Path** display on `/cmd_path`, set
  its Durability to *Transient Local*).
- **Motion is gated.** `motion_controller_node` always publishes what it *would* do on
  `/drive_intended`, but only sends real motion to `/drive` when **enabled** (default
  **HOLD**).

Boot the whole stack — graph, visualizer, and this console — with one command:
```bash
cd autonomous_rc_car && ./run.sh
```
(Or run just the console against an already-running graph, in its own terminal —
it needs a real TTY, so it can never be part of a launch file:)
```bash
ros2 run autonomous_rc_car_ros motion_enable_node
```
```
[HOLD] car: connected  | path:    14 wp | drive:      L0R0   (c=calibrate  p=plan  g=go  h/SPACE=hold  q=quit)
```
- **c** — calibrate against the phone's pose (Part 5). HOLDs first, then the car
  drives itself through the measurement sequence.
- **p** — compute a fresh path (watch it appear in RViz and the `wp` count update).
- **g** — GO: the car follows the current path; the `drive:` field shows the live
  `L..R..` command.
- **h** or **SPACE** — HOLD: commands `L0R0` immediately (SPACE = panic stop, and
  it aborts a running calibration).
- **q** — quit (also HOLDs).

The `car:` field is the ESP32 link. `disconnected` means nothing you press will
move the car — check power and `--car <ip>` first.

So you can't miss what it's doing: the console shows the path size and the exact drive
command in real time, and nothing moves until you press **g**. (Want auto-replanning
instead? `./run.sh --continuous`; arm at boot with `./run.sh --go`.)

## Part 4 — Getting `/drive` to the car (WiFi)

The laptop drives the ESP32 **directly over WiFi**. The phone stays a pure
perception sensor: no BLE, no iOS changes.

```
motion_controller_node ──/drive──► car_driver_node ──TCP :9001──► ESP32 ──► motors
```

### Flash the firmware

`esp32_firmware/src/esp32_car_wifi_tb6612.ino`, wired for **your** pins:

| | IN1 | IN2 | PWM |
|---|---|---|---|
| Motor A (left) | 18 | 19 | 23 |
| Motor B (right) | 16 | 17 | 21 |

STBY on **22**. Before flashing, fill in `WIFI_SSID` and `WIFI_PASS` at the top of
the sketch. On boot the serial monitor (115200) prints:

```
*** Car IP address: 192.168.1.xx
*** Listening on port 9001
```

### Protocol

Raw TCP, one command per line, port 9001:

- `L<left>R<right>\n` — each side −100..100, e.g. `L60R-40\n`; replies `ok\n`
- `S\n` — immediate stop
- 500 ms failsafe: no command → motors stop. The laptop sends at 10 Hz.

The firmware is deliberately **dumb** — a linear 0..100 → PWM map and nothing else.
Deadband, trim and every other per-car quirk live in the calibration file on the
laptop, so re-calibrating never means reflashing. Don't add compensation to the
firmware; it would fight the calibration.

### Point the laptop at it

```bash
./run.sh --car 192.168.1.xx        # or --car rccar.local (mDNS)
```

The console's `car:` field shows `connected` / `disconnected`. `car_driver_node`
reconnects on its own and stops the car whenever the socket drops.

### Bench test first

Wheels off the ground, then:

```bash
printf 'L60R60\n' | nc 192.168.1.xx 9001     # both sides forward
printf 'L0R0\n'   | nc 192.168.1.xx 9001     # stop
printf 'L60R-60\n' | nc 192.168.1.xx 9001    # spin in place
```

If a side runs backwards, flip `LEFT_DIR` / `RIGHT_DIR` in the sketch (or swap that
motor's two output wires) **before** calibrating.

---

## Part 5 — Calibration (press `c`)

Rather than guessing what a motor unit means, the car measures itself against the
iPhone's ARKit pose. Put the car on the floor with ~2×2 m clear around it, make
sure the phone is streaming and the car link says `connected`, then press **c**.

The console switches to `[CAL ]` and shows each stage:

1. **deadband** — ramps up in steps of 2 until the car actually moves, forwards
   and spinning. Finds the units below which stiction wins.
2. **turn sign** — spins and watches θ, so the car knows which way it rotates.
3. **angular gain** — spins at three speeds → rad/s per motor unit.
4. **straight runs** — three forward runs → m/s per unit, how much it veers, and
   how long it takes to react to a command.
5. **verification** — one more straight run with the trim applied.

Results are written to `config/calibration.yaml` and picked up **live** — the
driver and controller re-read it, no restart:

```yaml
turn_sign: 1
drive_deadband: 28         # motor units below which it does not move
turn_deadband: 24
linear_gain: 0.0042        # m/s per unit above the deadband
angular_gain: 0.0135       # rad/s per unit above the deadband
straightness_trim: -0.03   # +ve strengthens left, weakens right
command_latency: 0.18      # seconds from command to visible motion
```

`car_driver_node` applies the deadband and trim to every command; the controller
uses `turn_sign`, and uses `linear_gain × command_latency` as a lead distance so
the car stops **on** the waypoint instead of past it.

**Safety.** Calibration cannot run with the wheels off the ground — it measures
real motion. It aborts on stale pose, a lost car link, a stage timeout, or if the
car travels further than expected. Press **h** or **SPACE** to abort at any moment.
While it runs it owns the motors exclusively (`/calibration_active`), so the motion
controller cannot interfere.

Re-run it after changing wheels, battery voltage, or driving surface.

### If you would rather go BLE

`bridge_node` still relays `/drive` back over the phone socket as a framed message
(type `0x44 'D'`, ASCII `L..R..`), but the iOS app does not read it —
`CarController.swift` exists and nothing calls it. That path needs a read loop in
`PointCloudStreamer.swift` plus a `CarController` relay (plan Tasks 17–18). The
WiFi path above avoids all of it.

---

## Safety layers (once driving)
1. **Firmware failsafe** — the ESP32 stops if no command arrives for 0.5 s, so a
   crashed laptop, a dropped WiFi link or a killed graph all stop the car.
2. **Driver-side stop** — `car_driver_node` stops the car when the socket drops,
   and sends `S` on shutdown.
3. **HOLD by default** — `motion_controller_node` publishes `L0R0` unless you have
   pressed `g`; SPACE is the panic stop.
4. **Exclusive motor ownership** — `/calibration_active` means only one of
   calibration or the motion controller can command the car at a time.
5. **Calibration aborts** — stale pose, lost link, stage timeout, or travelling
   further than expected all stop the sequence.
6. `icp_slam_node` keeps the pose from drifting into the map.

Not yet wired: a PC-side stop when ARKit tracking degrades (the pose goes stale,
which the calibration catches, but the driving loop does not check it explicitly).
