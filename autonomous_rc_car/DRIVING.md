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
| ESP32 + TB6612 + motors, WiFi websocket control | ✅ **built and working** (user's own firmware) |
| **Laptop → ESP32 driver (`/drive` → ESP32 websocket)** | ⏳ **to build** — needs the ESP32's websocket command format (Part 4) |

So the **decision brain is complete end-to-end in software** and the car is built;
the one remaining link is a `car_driver_node` that forwards `/drive` to the ESP32
over WiFi (Part 4).

---

## Part 2 — Connect the ESP32 (hardware + firmware)

Parts and full wiring rules are in `Electronics-Shopping-List.md`. Firmware is
`esp32_firmware/src/esp32_car.ino` (L298N) and `esp32_car_tb6612.ino` (TB6612).

### Wiring (L298N — matches `esp32_car.ino`)

```
ESP32 GPIO25 ─► ENA   (left PWM)     ESP32 GPIO13 ─► ENB   (right PWM)
ESP32 GPIO26 ─► IN1                  ESP32 GPIO14 ─► IN3
ESP32 GPIO27 ─► IN2                  ESP32 GPIO12 ─► IN4
L298N OUT1/OUT2 ─► left motor        L298N OUT3/OUT4 ─► right motor
Motor battery (7.4 V 2×18650) ─► L298N 12V / GND
L298N GND ─► ESP32 GND               (COMMON GROUND — the #1 thing beginners miss)
ESP32 powered separately from its own USB battery to start.
```
TB6612 (recommended, cooler/efficient): wire per `Electronics-Shopping-List.md`
and flash `esp32_car_tb6612.ino` instead; the BLE protocol is identical.

### Flash it
- **Arduino IDE:** Boards Manager → install "esp32". Open `esp32_car.ino`, select
  your board, Upload. Serial Monitor @ 115200 should print
  `BLE robot car ready. Advertising as RobotCar-ESP32.`
- **PlatformIO:** the `esp32_firmware/platformio.ini` project is set up; `pio run -t upload`.

### BLE identity (must match the phone)
- Advertises as **`RobotCar-ESP32`**.
- Service UUID `6E400001-…`, RX write char `6E400002-…` (Nordic UART layout).
- These already match `CarController.swift`. Command char accepts WRITE /
  WRITE_NR of ASCII `L<left>R<right>` (e.g. `L60R-40`).

---

## Part 3 — Smoke-test and tune

### Bench test FIRST (wheels off the ground)
Prop the chassis so the wheels spin free. Prove BLE + motors before autonomy:
- Easiest: a generic BLE app ("nRF Connect", "BLE Controller – Arduino ESP32").
  Connect to `RobotCar-ESP32`, write `L60R60\n` to char `6E400002-…` → both sides
  forward. Write `L0R0\n` → stop. Write `L60R-60\n` → spins in place.
- The firmware has a **0.5 s failsafe**: if no command arrives for 500 ms it stops
  the motors. So the driver must send commands at least ~5×/s (the ROS
  `motion_controller` publishes at 10 Hz — good).

### Motor direction
If a wheel spins the wrong way, either swap that motor's two OUT wires, or flip its
sign in `setMotor` / the `IN1/IN2` order in firmware. Fix this before tuning.

### Tuning knobs

**In the ESP32 firmware (`esp32_car.ino`):**
- `PWM_FREQ` 20 kHz (silent) — leave as is.
- `FAILSAFE_MS` (500) — how long without a command before it stops.
- `map(mag, 0, 100, 0, 255)` — the speed→duty curve. Cheap TT motors have a
  **deadband** (don't turn below ~40–50% duty on 6–7 V). If the car stalls at low
  speeds, raise the floor, e.g. `map(mag, 0, 100, 90, 255)` so speed 1 already
  overcomes stiction.

**In the nav library (`laptop_brain/nav/config.py`) — the driving behavior:**
- `TURN_SIGN` (**calibrate first**): `+1` means command (left=+s, right=−s) makes
  heading θ increase. If the car turns the **wrong way** during autonomy, set it to
  `-1`. One-line fix, decided by watching one turn.
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
an editable install.)

### Bring-up order (never all at once)
1. Wheels off ground → BLE smoke test (`L60R60`) → confirm direction.
2. Add motor caps + a single **star ground**; retest (motion smooth, ESP32 never resets).
3. Mount the phone facing forward; drive manually.
4. Enable autonomy; calibrate `TURN_SIGN` on the first commanded turn.

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

Run the one control console (its own terminal) — it's also the status readout:
```bash
ros2 run autonomous_rc_car_ros motion_enable_node
```
```
[HOLD] path:   14 wp | drive:      L0R0   (p=plan  g=go  h/SPACE=hold  q=quit)
```
- **p** — compute a fresh path (watch it appear in RViz and the `wp` count update).
- **g** — GO: the car follows the current path; the `drive:` field shows the live
  `L..R..` command.
- **h** or **SPACE** — HOLD: commands `L0R0` immediately (SPACE = panic stop).
- **q** — quit (also HOLDs).

So you can't miss what it's doing: the console shows the path size and the exact drive
command in real time, and nothing moves until you press **g**. (Want auto-replanning
instead? launch the planner with `continuous:=true`; arm at boot with the controller's
`start_enabled:=true`.)

## Part 4 — Getting `/drive` to the car

**Recommended (your WiFi ESP32):** drive the ESP32 **directly from the laptop over
WiFi**. A `car_driver_node` subscribes to `/drive` and forwards each `L..R..` to the
ESP32's websocket (the same channel your HTML control page uses). The phone stays a
pure perception sensor; **no BLE and no iOS changes needed**. This is the intended
path now — tell me your ESP32's websocket command format and I'll write the node.

**Alternative (BLE via the phone):** if you ever go BLE instead, `bridge_node`
already relays `/drive` back over the socket as a framed message (type `0x44 'D'`,
ASCII `L..R..`), but **the iOS app does not yet read it.** `CarController.swift` can
drive the ESP32, but nothing calls it. Finishing that path needs (plan Tasks 17–18):

1. **Receive** in `PointCloudStreamer.swift`: a read loop on the TCP socket that
   parses inbound `type(1) + len(u32 LE) + payload` frames; on `0x44`, decode the
   `L..R..` ASCII.
2. **Relay**: instantiate a `CarController`, wait for `isConnected`, and on each
   decoded command call `carController.drive(left:right:)`.
3. **Safety**: call `carController.stop()` when ARKit tracking degrades or the
   socket drops (defense-in-depth with the ESP32's 0.5 s failsafe and the PC
   stale-pose stop).

**Alternative for testing without the phone relay:** drive the ESP32 **directly
from the laptop** with Python `bleak` (PC ⇄ BLE ⇄ ESP32), bypassing the phone —
useful to validate the motion controller against real motors before writing the
Swift code. The phone then only does perception.

---

## Safety layers (once driving)
1. ESP32 stops if no BLE command for 0.5 s (firmware).
2. PC should send `L0R0` if pose is stale or tracking ≠ normal.
3. Phone should `stop()` on tracking loss (Part 4, item 3).
4. `icp_slam_node` keeps the pose from drifting into the map.
