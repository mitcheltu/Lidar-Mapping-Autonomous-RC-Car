# Lowest-Cost iPhone-Brain Robot — Build Plan

**Goal:** A small vehicle that drives itself around your floor, using an
**iPhone 12 Pro Max** mounted on top as the "brain." The phone's LiDAR + camera
build a 3D point-cloud / map of the room (via Apple ARKit), decide where to go,
and send drive commands over Bluetooth to an **ESP32** that runs the motors.

**Your constraints (from our chat):** you already own an ESP32; it doesn't have
to look like a car, just something that moves across the floor; target parts
budget **$25–50**, with a **$50–120** option shown too; you're strong on
software but new to hardware.

---

## 1. The big idea and why it works

This is a real, proven architecture — you are not inventing it from scratch.

The closest working reference project is **RoBart** (open source, GitHub
`trzy/RoBart`): an iPhone is the entire brain of a rolling robot. It uses
**ARKit's 6-degrees-of-freedom pose tracking** to know where it is, runs the
navigation/control logic on the phone, and sends motion commands to a
microcontroller over **Bluetooth Low Energy (BLE)** — exactly the pattern we're
using, just on a cheaper chassis.

On the motor side, ESP32 + BLE + an L298N H-bridge driving two DC gear motors is
one of the most common beginner robot builds on the internet, so parts, wiring
diagrams, and troubleshooting help are everywhere.

The division of labor:

| Job | Who does it |
|---|---|
| See the room in 3D (LiDAR depth) | iPhone (ARKit `sceneDepth`) |
| Know where the robot is / has been | iPhone (ARKit world tracking, 6-DoF) |
| Build the point cloud / map | iPhone (the Swift app we built) |
| Decide where to drive | iPhone (control logic) |
| Turn decisions into motor power | ESP32 + H-bridge |
| Move | 2 DC gear motors + wheels |

**Key reality check on the phone's LiDAR:** the iPhone LiDAR is a *short-range*
sensor — reliable to roughly **5 meters**, and it points where the camera points
(forward/down). That's perfect for an indoor floor robot mapping a room and
avoiding furniture. It is *not* a spinning 360° LiDAR, so the robot "sees" in the
direction it faces. Mount the phone tilted slightly downward so it sees the floor
and obstacles right in front of the wheels.

---

## 2. What the research turned up

**iPhone-as-brain robots are a thing.** RoBart demonstrates the full stack:
ARKit pose → on-device navigation → BLE → microcontroller → motors, including a
PID controller fed by ARKit's 6-DoF pose for driving straight and turning
accurately. That's the blueprint we're following.

**ARKit gives you the point cloud for free-ish.** ARKit on LiDAR devices exposes
a per-frame depth map (`ARFrame.sceneDepth`) plus a confidence map, and separately
a live reconstructed **mesh** (`ARMeshAnchor`, "Scene Reconstruction"). You can
turn the depth map into a colored point cloud (what the app we built does) or use
the mesh directly for obstacle geometry. Apple's own WWDC sample "Visualizing a
Point Cloud Using Scene Depth" is the canonical example.

**ESP32 BLE robot cars are extremely well-trodden.** Countless tutorials drive a
2-motor car from a phone over BLE using an ESP32 and an L298N dual H-bridge.
There are even off-the-shelf iOS apps (e.g. "BLE Controller – Arduino ESP32",
Dabble) that send button commands to an ESP32 — useful for a **manual RC test**
before you write any autonomy code.

**Cheaper non-phone LiDAR alternatives exist** (e.g. a $15 Xiaomi LDS02RR 2D
spinning LiDAR wired to the ESP32, à la the `makerspet` / Linorobot2 / micro-ROS
projects). Worth knowing about, but since you *want* to use the iPhone you already
have, the phone-brain route avoids buying any LiDAR at all and gives you far more
compute for perception.

Sources are listed at the end.

---

## 3. Recommended architecture (the cheap, achievable path)

```
        ┌──────────────────────────┐
        │      iPhone 12 Pro Max     │
        │  ARKit LiDAR + camera      │
        │  • build point cloud/map   │
        │  • detect obstacles        │
        │  • decide drive commands   │
        └──────────────┬────────────┘
                       │  BLE  "L60R60\n"
                       ▼
        ┌──────────────────────────┐
        │          ESP32             │  (you already own this)
        │  • BLE server              │
        │  • parse L/R speeds        │
        │  • PWM + direction pins    │
        └──────────────┬────────────┘
                       │  6 wires
                       ▼
        ┌──────────────────────────┐
        │   L298N / TB6612 H-bridge  │
        └───────┬───────────┬───────┘
                ▼           ▼
             Left motor   Right motor      + caster/ball for the 3rd contact point
```

**Differential drive** (one motor per side, no steering servo) is the cheapest
and simplest way to move and turn: both forward = drive straight; opposite = spin
in place. That's why the chassis kits below use two motors and a caster.

Start it in phases so you always have something working:

1. **RC phase** — phone (or a free BLE app) sends manual drive commands; you
   verify motors, wiring, and BLE.
2. **Reactive-autonomy phase** — phone reads LiDAR depth straight ahead; if
   something's within ~0.5 m, stop and turn toward open space. Simple, robust,
   surprisingly effective.
3. **Mapping/navigation phase** — use the accumulated point cloud + ARKit pose to
   build a floor map and drive to chosen spots.

---

## 4. Bill of materials

Prices are typical US street prices (Amazon/AliExpress, mid-2026) and vary; buy
the cheapest reputable listing. You already have the ESP32 and the phone, so
those are $0 to you.

### Tier 1 — Absolute minimum ($25–50 target)

| Item | What / why | ~Price |
|---|---|---|
| 2WD robot chassis kit | Acrylic base + **2 TT gear motors** + 2 wheels + caster + battery box. This is the cheapest way to get motors, wheels, and a frame together. | $11–15 |
| L298N motor driver module | Dual H-bridge; drives both motors from ESP32. (TB6612FNG ~$5 is a more efficient swap.) | $5–7 |
| Jumper wires (M-M / M-F pack) | ESP32 ↔ L298N ↔ battery. | $4–6 |
| 18650 cells ×2 + holder **or** 4×AA holder | Motor power (6–8 V). | $6–10 |
| Phone mount / clamp | A cheap adhesive phone holder or a zip-tied clamp; tilt it slightly down. | $5–8 |
| Misc: zip ties, tape, small breadboard | Mounting + solderless wiring. | $3–5 |
| **ESP32** | You already own it. | $0 |
| **Tier 1 total** | | **≈ $34–51** |

You can hit the low end by buying a chassis kit that already bundles the L298N
and battery box (many do for ~$16–20), which removes two line items.

### Tier 2 — Low but noticeably better ($50–120)

Everything in Tier 1, plus upgrades that make driving and mapping more reliable:

| Upgrade | Why it helps | ~Added |
|---|---|---|
| 4WD chassis (4 motors) | More traction/torque on carpet and thresholds. | +$8–15 |
| TB6612FNG driver (instead of L298N) | Less voltage drop → motors get more power from the same battery; runs cooler. | +$0–5 |
| Wheel **encoders** + brackets | Measure actual wheel rotation → drive straight and turn by exact amounts (closed-loop). | +$6–12 |
| 7.4 V 2S LiPo + charger **or** better 18650 pack | More consistent power than AAs; longer runtime. | +$18–30 |
| Buck converter (e.g. MP1584) | Clean 5 V/3.3 V for the ESP32 from the motor battery. | +$4–7 |
| Sturdier phone cradle | Keeps the LiDAR pointed consistently — matters a lot for mapping. | +$8–15 |
| **Tier 2 total** | | **≈ $75–120** |

Optional cheap insurance for either tier: a small **HC-SR04 ultrasonic sensor**
($2–4) wired to the ESP32 as a last-ditch bump stop, independent of the phone.

---

## 5. Build it — step by step (hardware-new friendly)

**A. Assemble the chassis.** Screw the two TT motors into the acrylic base, press
on the wheels, attach the caster/ball at the front or back. Mount the battery box
underneath. (Kits include the screws and a diagram.)

**B. Mount the electronics.** Stick the ESP32 and the L298N on the top plate with
double-sided foam tape or a small breadboard. Leave room for the phone cradle.

**C. Wire it** (matches the pins in `autonomous_rc_car/esp32_firmware/src/esp32_car.ino`):

```
Battery (+6–8V) ──► L298N  12V  in
Battery (−/GND) ──► L298N  GND  ──► ESP32 GND     (common ground is essential)
L298N  5V out    ──► (optional) ESP32 5V, ONLY if the ESP32 isn't USB-powered

ESP32 GPIO25 ──► L298N ENA   (left speed / PWM)
ESP32 GPIO26 ──► L298N IN1
ESP32 GPIO27 ──► L298N IN2
ESP32 GPIO14 ──► L298N IN3
ESP32 GPIO12 ──► L298N IN4
ESP32 GPIO13 ──► L298N ENB   (right speed / PWM)

L298N OUT1/OUT2 ──► Left motor
L298N OUT3/OUT4 ──► Right motor
```

Notes for a first-timer:
- **Common ground** (battery −, L298N GND, ESP32 GND all connected) is the #1
  thing beginners miss. Without it nothing works.
- Power the **motors from the battery**, and power the **ESP32 from its own USB
  battery/cable** to start — simplest and safest. Only share power through a buck
  converter once things work.
- If a motor spins the "wrong" way, swap its two OUT wires (or flip its sign in
  code). No harm done.
- L298N wastes ~2 V as heat; with 4×AA (6 V) the motors are a bit weak. 2×18650
  (7.4 V) or a TB6612 driver fixes this.

**D. Flash the ESP32.** In the Arduino IDE, add ESP32 boards (Boards Manager →
"esp32"), open `autonomous_rc_car/esp32_firmware/src/esp32_car.ino`, select your board, and Upload. Open the
Serial Monitor at 115200; you should see `BLE robot car ready.`

**E. Smoke-test the link.** Before autonomy, prove BLE works. Either build the
iOS app (below) and add a couple of manual buttons that call
`CarController.drive()`, or use a generic BLE app to write `L60R60\n` to the
characteristic and watch the wheels spin. Then `L0R0\n` to stop. The firmware
also has a **0.5 s failsafe** that stops the motors if commands stop arriving.

**F. Mount the phone.** Clamp the iPhone on top, camera facing forward and tilted
~15–25° down so the LiDAR sees the floor and near obstacles. Keep the lens
unobstructed.

---

## 6. The software you already have

Two pieces live in this project:

**`autonomous_rc_car/ios_app/RCCarLidarStreamer/` — the perception app.** SwiftUI + ARKit. Wave the phone
and it accumulates a colored 3D point cloud from the LiDAR, shows it live, and
exports `.PLY`. Setup instructions are in `autonomous_rc_car/ios_app/RCCarLidarStreamer/README.md`. This is
your mapping tool and the foundation for on-robot perception.

**`autonomous_rc_car/esp32_firmware/src/esp32_car.ino` — the motor controller.** BLE server that accepts
`L<left>R<right>` speed commands (−100…100) and drives the H-bridge with PWM,
with a motion failsafe.

**`autonomous_rc_car/ios_app/RCCarLidarStreamer/CarController.swift` — the bridge.** iOS BLE client that
finds the ESP32, connects, and exposes `drive(left:right:)` / `stop()`. The UUIDs
already match the firmware.

---

## 7. From "moves" to "drives itself" — the autonomy roadmap

Do these in order; each is useful on its own.

1. **Manual RC (day 1).** Wire `CarController` to on-screen buttons (or a generic
   BLE app). Confirm forward/back/turn and the failsafe.

2. **Straight-line + turn-by-angle (closed loop).** ARKit's 6-DoF pose
   (`frame.camera.transform`) tells you heading and position. Use it to hold a
   straight heading and to turn exactly 90°, the way RoBart uses a PID controller
   on ARKit pose. Encoders (Tier 2) help but ARKit pose alone gets you far.

3. **Reactive obstacle avoidance.** Each frame, read the center region of
   `sceneDepth`. If the nearest points ahead are closer than ~0.5 m, stop and
   rotate toward whichever side has larger depth (more open space), then resume.
   This is simple and robust and needs no map.

4. **Map + navigate.** Feed the accumulating point cloud (or the ARKit mesh) into
   a top-down occupancy grid: project points near floor height into 2D cells,
   mark cells with obstacles as blocked. Then pick a goal cell and drive toward it
   with a basic planner (even "turn toward goal, drive while clear" works). ARKit
   keeps the robot localized within the map as it moves.

A pragmatic control loop on the phone: 5–10 times per second, look at the depth
in front, compute a desired left/right speed, and call
`carController.drive(left:right:)`. Keep speeds low (30–60) indoors.

---

## 8. Pitfalls and tips

- **LiDAR is short range and directional.** ~5 m max, sees where the camera
  points. Tilt the phone down; don't expect it to see behind the robot.
- **Keep it slow.** ARKit tracking and depth degrade with fast motion and in
  low-texture / low-light rooms. Good lighting helps tracking a lot.
- **Battery sag is the classic gotcha.** Weak/old AAs make motors stutter and can
  brown-out the ESP32. Use fresh cells or 18650/LiPo, and give the ESP32 its own
  clean supply.
- **Thermal throttling.** Running ARKit continuously warms the phone; for long
  sessions keep it out of a hot case and the sun.
- **ESP32 core version.** The firmware assumes the current ESP32 Arduino core
  (3.x), where `getValue()` returns an Arduino `String`. On older 2.x cores it
  returns `std::string` — if it won't compile, either update the core or change
  that one line to read the value as bytes.
- **iOS permissions.** Add `NSCameraUsageDescription` (and
  `NSBluetoothAlwaysUsageDescription` once you use `CarController`) or the app
  crashes on launch.
- **Test on blocks first.** Prop the wheels off the ground for the first BLE test
  so a wiring mistake doesn't send it off a table.

---

## 9. Cost summary

| Build | You already own | You buy | Approx cash |
|---|---|---|---|
| **Tier 1 (minimum)** | iPhone, ESP32 | chassis+motors, driver, wires, battery, mount | **$34–51** |
| **Tier 2 (better)** | iPhone, ESP32 | + 4WD, encoders, LiPo, buck, better cradle | **$75–120** |

Cheapest realistic path to a self-driving floor robot: **~$35**, because the two
most expensive parts of any robot — the compute/sensing (iPhone) and the
microcontroller (ESP32) — you already have.

---

## 10. Sources

- RoBart — iPhone-brain autonomous robot (ARKit 6-DoF + BLE to microcontroller): https://github.com/trzy/RoBart
- ARKit & LiDAR: Building Point Clouds in Swift: https://medium.com/@ivkuznetsov/arkit-lidar-building-point-clouds-in-swift-2c9b7eb88b03
- ARKit 911 — Scene Reconstruction with a LiDAR Scanner (ARMeshAnchor): https://medium.com/macoclock/arkit-911-scene-reconstruction-with-a-lidar-scanner-57ff0a8b247e
- ios-depth-point-cloud (depth capture + PLY export, WWDC20-10611 based): https://github.com/Waley-Z/ios-depth-point-cloud
- ESP32 BLE robot car tutorial: https://www.robotique.tech/robotics/control-a-robot-car-based-on-esp32-by-bluetooth/
- Bluetooth-controlled ESP32 car (L298N wiring): https://www.hackatronic.com/bluetooth-controlled-car-using-esp32-and-android-phone/
- BLE Controller – Arduino ESP32 (iOS test app): https://apps.apple.com/us/app/ble-controller-arduino-esp32/id6754522781
- Cheap 2D LiDAR + ESP32 alternative ($15 LDS02RR): https://makerspet.com/blog/how-to-connect-xiaomi-lds02rr-lidar-to-esp32/
- Building the cheapest ROS2 robot using ESP32: https://robofoundry.medium.com/building-cheapest-ros2-robot-using-esp32-part-1-hardware-build-af0044de68ce
- 2WD chassis kit with TT motors + L298N (example): https://www.amazon.com/LAFVIN-Chassis-Ultrasonic-Compatible-Arduino/dp/B07YCHCQNK
