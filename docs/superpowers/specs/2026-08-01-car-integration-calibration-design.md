# Car Integration + Calibration — Design

**Date:** 2026-08-01
**Status:** approved, implementing

## Problem

The decision brain is complete in software but nothing reaches the motors:
`DRIVING.md` Part 4 lists `car_driver_node` as the one missing link. And even once
commands reach the car, the driving constants in `nav/config.py` (`TURN_SIGN`,
speeds) are guesses — nobody has measured what a motor unit does on this chassis.

## Hardware (given)

TB6612FNG, ESP32:

| | IN1 | IN2 | PWM |
|---|---|---|---|
| Motor A (left) | 18 | 19 | 23 |
| Motor B (right) | 16 | 17 | 21 |

STBY 22. These differ from `esp32_car_tb6612.ino` (STBY 33, 25/26/27, 13/16/17),
so that sketch does not match this car.

## Layer A — the car link

### Firmware: `esp32_firmware/src/esp32_car_wifi_tb6612.ino`

- WiFi **station mode**; `WIFI_SSID` / `WIFI_PASS` constants at the top of the
  sketch for the user to fill before flashing.
- TCP line server on **port 9001**, mDNS `rccar.local`, prints its IP on serial.
- Accepts `L<left>R<right>\n`, each side −100..100; replies `ok\n` so the driver
  can measure link RTT independently of mechanical latency.
- 20 kHz PWM, 8-bit, linear `0..100 -> 0..255`.
- 500 ms failsafe: no command → stop.
- Deliberately **dumb**: no deadband, no trim, no ramping. Those are laptop-side,
  so calibration measures raw hardware and can be re-run without reflashing.

### `car_driver_node`

`/drive` (String `L..R..`) → TCP socket. Applies the calibration mapping:
deadband floor, then straightness trim. Auto-reconnects; publishes `/car_link`
(String) with connection state and RTT. Params: `host` (default `rccar.local`),
`port` (9001), `timeout`.

## Layer B — calibration

### `calibration_node`

Measures against `/pose` — **raw ARKit VIO, not `/pose_corrected`**: ICP applies
discrete jumps that would corrupt short-move measurements.

Stages, each followed by a stop and a 0.5 s settle:

1. **Preflight** — pose fresh, link up, else abort.
2. **Deadband** — ramp both sides 0→up in steps of 2, 0.6 s per step, until
   displacement > 3 cm → `drive_deadband`. Repeat with opposite signs until
   |Δθ| > 5° → `turn_deadband`.
3. **Turn sign** — spin ~1.2 s; `turn_sign = +1` if θ increased, else −1.
4. **Angular gain** — spin at three speeds; least-squares fit rad/s per unit.
5. **Linear gain, straightness, latency** — drive straight at three speeds:
   displacement/time → m/s per unit; heading drift → `straightness_trim`; time
   from command to 20% of steady speed → `command_latency`. One verification run
   after the trim is applied.

Abort on: stale pose, displacement past a runaway limit, stage timeout, link loss,
or the operator pressing `h`/SPACE.

### Motor ownership

In HOLD, `motion_controller_node` publishes `L0R0` to `/drive` at 10 Hz (observed).
That would fight every calibration command. So `calibration_node` publishes a
latched `/calibration_active` (Bool), and `car_driver_node` obeys **only**
`/drive_raw` while it is true and **only** `/drive` otherwise. Exactly one owner of
the motors at any instant.

### Results

`autonomous_rc_car/config/calibration.yaml`, loaded at runtime by `nav.config`
(no rebuild to apply):

```yaml
calibrated_at: 2026-08-01T12:00:00
turn_sign: 1
drive_deadband: 28
turn_deadband: 24
linear_gain: 0.0042        # m/s per motor unit above deadband
angular_gain: 0.0135       # rad/s per motor unit above deadband
straightness_trim: -0.03   # +ve = right side stronger
command_latency: 0.18      # seconds
```

Applied automatically: `turn_sign` (controller), `drive_deadband` /
`straightness_trim` (car_driver_node). `linear_gain` and `command_latency` give the
controller a lead distance so it stops on the waypoint rather than past it;
`angular_gain` is reported and available.

## Console

`motion_enable_node` gains `c` = start calibration (publishes `/calibrate_trigger`).
The status line shows the live stage from `/calibration_status`. `h`/SPACE aborts
calibration as well as stopping motion.

## Code layout

Follows the existing split: `nav/calibration.py` holds the pure math — deadband
detection, least-squares gain fitting, trim, YAML load/save — with hardware-free
unit tests. The ROS nodes stay thin wrappers.

## Safety

- Pose-based calibration **cannot run with the wheels off the ground**; it needs
  roughly 2×2 m of clear floor and drives forward ~1.5 m in stage 5.
- Nothing moves until the operator presses `c`. HOLD remains the default.
- Command magnitudes are clamped; every stage has a timeout; the ESP32's 500 ms
  failsafe is the last line of defence.

## Verification

- `nav/calibration.py`: pytest, TDD, hardware-free (synthetic pose traces).
- `car_driver_node`: exercised against a stub TCP server in WSL — no ESP32 needed
  to prove framing, reconnect, and the `/drive` vs `/drive_raw` arbitration.
- Firmware and the physical calibration run need the real car; the user flashes and
  confirms the link before Layer B means anything.

## Out of scope

- BLE and the iOS relay: the phone stays a pure perception sensor.
- Nav2 integration.
