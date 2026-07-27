# TODO & Implementation Plan — Autonomous RC Car

Companion to `PROJECT_STATUS.md`. **Canonical architecture: ROS2 + Nav2** on the
laptop (decided 2026-07-27); the proven Python `nav/` package is the algorithm
library the ROS2 nodes wrap.

Last updated: **2026-07-27**.

---

## Summary — where things stand

**Done (works today):**
- iPhone LiDAR scanner + live Wi-Fi stream (points / pose / JPEG).
- Laptop `nav/` **algorithm library**, unit-tested: occupancy grid, cloud
  cleaning, floor estimation, grid + inflation, localization, frontier selection,
  A\* + path simplification, overlay geometry.
- `pc_viewer.py` — the **canonical viewer** (live 3D cloud, pose marker, camera
  feed, disk recording, `G`-key walkthrough preview). Redundant `live_viewer.py`
  removed.
- Shared wire protocol module `nodes/stream_protocol.py` (tested).
- ESP32 firmware written (`esp32_car.ino` + TB6612 variant).

**Not done (the work ahead):**
- The **ROS2 layer is name-only** — no `rclpy`, no ament package, launch/config
  are stubs (see `PROJECT_STATUS.md` §7.2). Must be built for real.
- No autonomy loop runs; nothing drives.
- Car hardware not built; firmware never flashed/field-tested.
- Repo not yet committed (disk full); docs have stale paths; iOS app duplicated.

**Critical path:** free disk → git baseline → make ROS2 real → mapping/SLAM nodes
on recorded data → planning+control → iOS reverse channel → build car → field test.

---

## ✅ Done — Milestone A: phone-only mapping library

- [x] `nav/` package + pytest harness (Tasks 1–8, 15, 15b of the TDD plan)
  - [x] `OccupancyGrid` (world/cell mapping, passable mask)
  - [x] cloud cleaning (voxel downsample + statistical/radius outlier removal)
  - [x] floor-height estimation (y-histogram)
  - [x] height-banded occupancy grid + EDT inflation
  - [x] localization: pose matrix → `(x, z, θ)`
  - [x] frontier detection + BFS reachability + nearest-frontier goal
  - [x] A\* (no corner cutting) + line-of-sight simplification
  - [x] viewer overlay geometry + `G`-key walkthrough preview
- [x] iPhone scanner app: live cloud, PLY export, Wi-Fi stream
- [x] ESP32 firmware written (both L298N and TB6612 variants)
- [x] Viewer consolidation: `live_viewer.py` removed, `pc_viewer.py` canonical

---

## Phase 0 — Repo baseline & housekeeping (do first; unblocks everything)

- [ ] **Free disk space** — `C:` is 100% full; blocks `git` and `pip`. Reclaim
      `.venv312/` (721 MB) / `captures/` (216 MB) / `.venv/` (22 MB) as you see fit.
- [ ] **Write `.gitignore`** at repo root: `.venv/`, `.venv312/`, `__pycache__/`,
      `.pytest_cache/`, `captures/`, `*.f32`, stray `*.ply`, `tmp_*`.
- [ ] **Git baseline** — treat the reconciled tree as the start of the repo:
      `git add -A` the `autonomous_rc_car/` structure + the root docs, make the
      **initial commit**. (Old `viewer/`/`firmware/`/`PointCloudScanner/` are gone.)
- [ ] **Fix stale doc paths** — `viewer/`→`laptop_brain/`, `firmware/`→
      `esp32_firmware/`, `PointCloudScanner/`→`ios_app/RCCarLidarStreamer/` across
      `Build-Plan.md`, the ios_app README, and the TDD plan.
- [ ] **De-dup the iOS app** — reconcile `RCCarLidarStreamer` vs "PointCloudScanner"
      naming; remove duplicated Swift files (root `ContentView.swift` +
      `App/ContentView.swift`; `PointCloudStreamer.swift` + `Network/*Streamer.swift`).

## Phase 1 — Make ROS2 real (the foundation)

- [ ] **Decide the ROS2 host** — ROS2 on Windows is painful; pick **WSL2/Ubuntu**
      or a **Docker** dev container for `laptop_brain`. Document the choice + setup.
- [ ] **Create an ament_python package** for `autonomous_rc_car`: `package.xml`,
      `setup.py`, `setup.cfg`, `resource/`, console_scripts entry points. Make
      `colcon build` + `ros2 run` actually work.
- [ ] **Define the topic graph & messages** (reuse std/nav/sensor msgs where
      possible): `/points` (PointCloud2), `/pose` + `/pose_corrected`
      (PoseStamped), `/image` (CompressedImage), `/map` (OccupancyGrid),
      `/cmd_path` (Path), `/drive` (custom L/R or Twist).
- [ ] **Convert `nodes/*.py` from scripts to real `rclpy` nodes** (keep them thin;
      the heavy logic stays in `nav/`):
  - [ ] `bridge_node` — TCP server for the iPhone stream → publish `/points`
        `/pose` `/image`; subscribe `/drive` → send back over the reverse channel.
        (Extend `stream_protocol.py` to cover pose + JPEG so there's one protocol.)
  - [ ] `voxel_mapper_node` — subscribe `/points` `/pose`, call `nav.mapping`
        (clean → floor → grid → inflate), publish `/map`.
  - [ ] `frontier_planner_node` — subscribe `/map`, call `nav.frontier` +
        `nav.planner`, publish `/cmd_path`. (Or delegate to Nav2 — see Phase 3.)
  - [ ] `motion_controller` — subscribe `/cmd_path` `/pose_corrected`, run
        pure-pursuit/waypoint follow, publish `/drive`.
  - [ ] `icp_slam_node` — subscribe `/points` `/pose`, run KISS-ICP scan-to-map,
        publish `/pose_corrected`.
- [ ] **Wire the config yamls** — actually load `calibration_params.yaml`
      (extrinsics, robot radius, TURN_SIGN), `kiss_icp_params.yaml`; delete or
      populate `nav2_params.yaml` per the Phase-3 decision.
- [ ] **Rewrite `launch/bringup.launch.py` + `mapping.launch.py`** to launch the
      real nodes with params (not `executable="python"` + a script path).

## Phase 2 — Mapping & SLAM on recorded data (hardware-free)

- [ ] Replay a `captures/<session>/points_log.f32` through `bridge_node` (or a
      replay node) so the whole mapping graph runs with **no phone/car needed**.
- [ ] Validate `voxel_mapper_node` `/map` against the `nav/` unit tests' behavior.
- [ ] Integrate **KISS-ICP** in `icp_slam_node`; confirm drift correction on a
      recorded walkthrough (compare raw vs corrected trajectory).
- [ ] Point `pc_viewer.py` at the ROS2 topics (or keep it on the raw stream) and
      overlay `/map` + `/cmd_path`. Move the preview off the GUI thread.

## Phase 3 — Planning & control

- [ ] **Nav2 decision:** full Nav2 bringup (costmap_2d + planner + controller +
      bt_navigator, driven by frontier goals) **vs.** the lightweight custom
      `nav.frontier` + `nav.planner` + `motion_controller`. Nav2 is the canonical
      target; the custom stack is the fallback if Nav2 is too heavy. Record which.
- [ ] If Nav2: publish `/map` as a proper costmap, provide the robot TF tree
      (`map`→`odom`→`base_link`), configure `nav2_params.yaml`, feed frontier
      goals to `bt_navigator`.
- [ ] **Reverse command channel** — finalize DRIVE/MODE frames PC→phone (the TDD
      plan's `0x44 'D'` / `0x4D 'M'`), with the redundancy layers (ESP32 0.5 s
      failsafe + PC stale-pose stop + phone tracking-loss stop + command heartbeat).
- [ ] **Performance follow-ups** before scaling up: vectorize/offload
      `bfs_distances` (~4.7 s worst case on 400×400 open grids); fix
      `simplify_path` super-quadratic maze worst case.

## Phase 4 — iOS bidirectional (Swift)

- [ ] Streamer receives DRIVE/MODE, relays DRIVE to the ESP32 via `CarController`
      (BLE), gates LiDAR point capture on SCAN mode, sends tracking status to PC.
- [ ] Tracking-loss safety stop: phone calls `carController.stop()` itself when
      ARKit tracking degrades (works even if the PC link dies).

## Phase 5 — Hardware bring-up & field test

- [ ] Buy parts (see `Electronics-Shopping-List.md`): 4WD chassis (TT motors +
      encoders + 18650 holder), 2× TB6612FNG, 2× 18650 + charger, caps, phone clamp.
- [ ] Assemble chassis; wire drivers (separate supplies, one common star ground,
      motor caps). Flash ESP32; `L60R60` BLE smoke test (wheels off the ground).
- [ ] Mount phone facing forward; manual RC drive over BLE.
- [ ] **Field test & calibration** (TDD plan Task 19): set `TURN_SIGN`, verify all
      four redundancy layers, run reactive avoidance → frontier mapping.

---

## Notes / risks

- **ROS2 on Windows** is the biggest friction point — resolve the host decision
  (Phase 1) before investing in node conversion.
- **Nav2 vs custom planner** is a real fork: Nav2 is powerful but a big dependency
  and TF/costmap learning curve; the tested `nav/` stack already does frontier +
  A\* and could drive the car with far less overhead. Decide early (Phase 3).
- The `nav/` library is the crown jewel — **don't discard it** when adopting ROS2;
  wrap it.
