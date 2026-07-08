# Electronics Shopping List & Wiring Guide
### iPhone-LiDAR autonomous mapping rover — **4-Wheel Drive**

**Architecture:** iPhone 12 Pro Max (ARKit LiDAR mapping + navigation) → **BLE** →
ESP32 (motor control) → 2× motor driver → 4 motors. Your **power bank** runs the
ESP32 (clean logic power); a **separate 18650 battery** runs the motors. That
power separation, a single common ground, and a few noise capacitors are what
make the electronics reliable.

> **Raspberry Pi:** not needed for this build — the iPhone talks directly to the
> ESP32 over Bluetooth. Leave the Pi off the robot. (Optional telemetry box later.)

Prices are typical US street prices (mid-2026) and move around; treat them as
ballpark. Links are example listings, not the only option — buy the cheapest
reputable equivalent.

---

## Already have

| Item | Role |
|---|---|
| iPhone 12 Pro Max | Brain — forward-facing LiDAR mapping + navigation |
| ESP32 | Motor controller (BLE server, runs the firmware) |
| Power bank + USB cable | Logic power for the ESP32 (isolated from motor noise) |
| Breadboard + DuPont wires | Logic/signal wiring only — **not** motor current |
| Raspberry Pi | Not used here |

---

## Buy — essentials (4WD)

Roughly **$63–99** total for the essentials.

| # | Part | Recommended pick (example link) | Notes | ~Price |
|---|---|---|---|---|
| 1 | **4WD chassis kit** — 4 TT gear motors (with wires) + encoders + 18650 holder | [C101 4WD w/ speed-encoder TT motors + 2×18650](https://www.amazon.com/C101-Robotics-Platform-Raspberry-Eduational/dp/B09V7CJT36) · pre-wired alt: [OSOYOO FlexiRover (motors soldered w/ 2-pin leads, 18650 holder + switch)](https://www.amazon.com/OSOYOO-FlexiRover-Building-Kit-Arduino/dp/B0DJ7BT1V5) · [LK COKOINO 4WD (2×18650 case)](https://www.amazon.com/Arduino-LK-COKOINO-Raspberry-Building/dp/B0B5JPJ9R4) | Gives frame + 4 wheels + 4 motors + battery box. Pick one whose motors **come with wires** to avoid soldering. Confirm it takes **2×18650** (not 4×AA). | $22–38 |
| 2 | **2× TB6612FNG driver** (with headers) | [SparkFun Dual TB6612FNG w/ headers (Amazon)](https://www.amazon.com/SparkFun-Motor-Driver-TB6612FNG-Headers/dp/B07PV1S8HX) · [same at SparkFun](https://www.sparkfun.com/sparkfun-motor-driver-dual-tb6612fng-with-headers.html) | **Two** boards for 4WD — one per side, so each motor gets its own channel (cool + reliable). Cheaper generic TB6612 multipacks work too. "With headers" = no soldering to breadboard. | $12–18 |
| 3 | **2× 18650 cells + charger** | [18650 button-top + charger kits (Amazon)](https://www.amazon.com/18650-rechargeable-battery-charger/s?k=18650+rechargeable+battery+with+charger) | Reputable brand (EBL, XTAR, Nitecore), protected, ~3000 mAh. Match **button-top vs flat-top** to your holder. Stable power = the #1 reliability factor. | $14–22 |
| 4 | **Capacitor assortment** (0.1 µF ceramics + a few 100–470 µF electrolytics) | [Ceramic capacitor assortment kit (Amazon search)](https://www.amazon.com/s?k=ceramic+capacitor+assortment+kit) | Kills motor brush noise + voltage dips that crash the ESP32/BLE. Many TT motors already have a cap fitted (see soldering note). | $6 |
| 5 | **Phone clamp mount** | [KAMISAFE tripod phone clamp (fits iPhone 12 Pro Max)](https://www.amazon.com/KAMISAFE-Tripod-Phone-Mount-Holder/dp/B0CNGMRTRQ) · [NEEWER clamp](https://www.amazon.com/Neewer-Smartphone-Holder-Vertical-Bracket/dp/B075R229KH) | Mount it **facing forward**. These have a 1/4" tripod screw — attach to the chassis with a small 1/4" bracket/bolt (or epoxy a base). | $9–15 |
| 6 | **Power switch** | usually included in the chassis kit | Clean on/off for the motor battery. | $0–3 |

## Buy — recommended / situational

| Part | When | Example | ~Price |
|---|---|---|---|
| **Beginner soldering iron kit** | For the motor caps + tinning wire ends (see below). Skip if you have one. | [Beginner soldering iron kits (Amazon search)](https://www.amazon.com/s?k=beginner+soldering+iron+kit) | $15–25 |
| **HC-SR04 ultrasonic sensor** | Independent close-range "don't hit the wall" stop — covers the forward-facing camera's near-floor blind spot. | [HC-SR04 (Amazon)](https://www.amazon.com/hc-sr04/s?k=hc-sr04) | $2–5 |
| **Buck converter (MP1584/LM2596)** | Only if you'd rather power the ESP32 from the 18650 pack instead of the power bank. Not needed with the power bank. | Amazon "MP1584 buck" | $4–7 |

---

## Will I need to solder?

You can go **solder-free** by choosing: motors **with wires** (kit #1), TB6612
boards **with headers** (kit #2), a **screw-terminal** or button-top battery
holder, and DuPont jumpers for all logic. Many TT motors even ship with a
**noise cap already soldered** across the terminals, so #4 may be optional.

For a robot that rattles around the floor, I still suggest a cheap iron for ~6–10
easy joints (motor caps + tinning power leads) — soldered joints won't shake loose
mid-drive. Fully optional; the build works without it.

---

## 4WD wiring — differential drive with two drivers

Four motors, **two per side**, steered like a tank (both sides forward = straight;
opposite = spin in place, which is how it does the 360° scans).

```
POWER BANK ──USB──► ESP32                (logic power)

18650 pack (7.4V) ─► SWITCH ─┬─► Driver-L  VM
                             └─► Driver-R  VM
All grounds (battery −, ESP32 GND, both driver GNDs) ─► ONE common star point.

ESP32 3.3V ─► both drivers VCC + STBY (enable)

Driver-L (left side):  AO ─► left-front motor,  BO ─► left-rear motor
Driver-R (right side): AO ─► right-front motor, BO ─► right-rear motor

ESP32 GPIOs ─► Driver-L AIN1/AIN2/PWMA + BIN1/BIN2/PWMB
ESP32 GPIOs ─► Driver-R AIN1/AIN2/PWMA + BIN1/BIN2/PWMB
```

Simpler one-driver option (hard floors, gentle use): a **single** TB6612 with the
two left motors wired in parallel to one channel and the two right to the other.
Works, but two paralleled TT motors can exceed the TB6612's 1.2 A/channel when
stalling on carpet — that's why two boards is the "flawless" pick.

Either way the firmware only ever thinks in **two logical sides** (left/right), so
the `L..R..` BLE command format is unchanged. *(Ask me for the two-board firmware
variant and I'll wire up the extra pins.)*

### Five rules that make it reliable
1. **Separate supplies, one common ground** (star point near the battery). Most important rule.
2. **100 nF ceramic across each motor's terminals**, close to the motor body.
3. **Bulk cap (100–470 µF) across VM→GND** at each driver.
4. **Motor current never goes through the breadboard** — straight to driver terminals.
5. **Motor wires short + twisted**, routed away from the ESP32 antenna.

---

## Trusted tutorials

- Connect TB6612 to ESP32 (schematic + code): https://makerspet.com/blog/connect-tb6612fng-motor-driver-to-esp32/
- DroneBot Workshop — TB6612FNG H-Bridge (why it beats L298N): https://dronebotworkshop.com/tb6612fng-h-bridge/
- SparkFun TB6612FNG Hookup Guide: https://learn.sparkfun.com/tutorials/tb6612fng-hookup-guide/all
- Random Nerd — ESP32 DC motor speed/direction + PWM: https://randomnerdtutorials.com/esp32-dc-motor-l298n-motor-driver-control-speed-direction/
- Pololu — Dealing with Motor Noise (capacitor placement): https://www.pololu.com/docs/0J15/9
- Robots for Roboticists — Grounding & avoiding ground loops: https://www.robotsforroboticists.com/grounding-avoiding-ground-loops/

---

## Bring-up order (never all at once)

1. Wheels **off the ground**: power bank → ESP32, 18650 → drivers, flash firmware, send `L60R60` over BLE, confirm all four wheels spin the right way (flip a motor's two leads if reversed).
2. Add caps + star ground; retest — motion smooth, ESP32 never resets.
3. Assemble chassis; mount phone **facing forward**.
4. Drive manually over BLE.
5. Bring up autonomy (reactive avoidance → frontier mapping) on the phone.

### Cost summary
| Tier | Buy | Approx |
|---|---|---|
| Essentials (#1–6) | 4WD chassis, 2× TB6612, 18650×2+charger, caps, mount, switch | **$63–99** |
| + niceties | soldering kit, ultrasonic failsafe, buck converter | +$21–37 |

Because the iPhone and ESP32 are already yours, a rock-solid 4WD mapping rover
lands around **$65–100** in new parts.
