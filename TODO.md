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

## Phase 0 — Repo baseline & housekeeping (mostly DONE)

- [x] **Free disk space** — user reclaimed space; ~59 GB free now.
- [x] **Write `.gitignore`** at repo root for the new tree (venvs, captures, `*.f32`,
      `tmp_*`, ROS2 build/install/log).
- [x] **Git baseline** — committed the reorg (history preserved, 29 renames detected)
      as `28d45f1` on `milestone-a-nav`. 50 tests green beforehand.
  - [ ] **Push blocked** — `git push` fails with an SSL cert error (env/proxy issue).
        Retry with `! git push -u origin milestone-a-nav` once the cert/network is fixed.
  - [ ] Optional: merge `milestone-a-nav` → `master`.
- [x] **Fix stale doc paths** — done in `Build-Plan.md` + ios_app README (TDD plan
      left as historical).
- [x] **De-dup the iOS app** — removed the stub `App/ AR/ Network/` folders (they were
      placeholders with a second `@main`). Folder is now 7 real Swift files, one `@main`.
      README file list + permission keys fixed (added `PointCloudStreamer.swift`,
      `NSLocalNetworkUsageDescription`). *(struct still named `PointCloudScannerApp` —
      cosmetic, not a build issue.)*

## Phase 1 — Make ROS2 real (the foundation)

- [x] **ROS2 host decided: WSL2 + Ubuntu 22.04 / ROS2 Humble** (2026-07-27).
- [x] **Layout decided: Hybrid** — `nav/` stays the pip-installable tested library;
      a thin ament package `autonomous_rc_car_ros` (under `ros2_ws/src/`) holds the
      rclpy nodes that import `nav.*`.
- [x] **Made `nav/` pip-installable** — `laptop_brain/pyproject.toml` (`rc-car-nav`),
      moved the wire protocol into `nav/stream_protocol.py` (nodes/ shim kept). 57 tests green.
- [x] **Wrote the WSL2/ROS2 setup guide** — `autonomous_rc_car/ROS2_SETUP.md`
      (install, build/run, and the WSL2 networking fix so the phone can reach `bridge_node`).
- [x] **Scaffolded the ament package** `ros2_ws/src/autonomous_rc_car_ros` (package.xml,
      setup.py, entry points, `bridge_node` real + 4 stub nodes, launch, README).
      Syntax-checked on Windows; **NOT yet colcon-built.**
- [ ] **← NEXT MAJOR-VALIDATION CHECKPOINT:** in WSL2, `pip install -e laptop_brain`
      + `colcon build` + `ros2 launch ... bringup.launch.py`; confirm the phone stream
      reaches `bridge_node` and `/points` `/pose` `/image` publish. Report back before
      the 4 stub nodes are implemented.
- [ ] **Define the topic graph & messages** (reuse std/nav/sensor msgs where
      possible): `/points` (PointCloud2), `/pose` + `/pose_corrected`
      (PoseStamped), `/image` (CompressedImage), `/map` (OccupancyGrid),
      `/cmd_path` (Path), `/drive` (custom L/R or Twist).
- [ ] **Convert `nodes/*.py` from scripts to real `rclpy` nodes** (keep them thin;
      the heavy logic stays in `nav/`):
  - [x] Unified wire protocol: `stream_protocol.py` now covers points/pose/image;
        `pc_viewer.py` uses it (one protocol for the viewer + future bridge). ✅
  - [ ] `bridge_node` — TCP server for the iPhone stream → publish `/points`
        `/pose` `/image`; subscribe `/drive` → send back over the reverse channel.
        (Reuse `stream_protocol.py` for framing.)
  - [ ] `voxel_mapper_node` — subscribe `/points` `/pose`, call `nav.mapping`
        (clean → floor → grid → inflate), publish `/map` (OccupancyGrid). Also
        publish **categorized layers for visualization**: occupied / ground(floor) /
        free as separate `PointCloud2` (or a cube `MarkerArray`) topics so they can
        be toggled independently — this is the "see the voxels" capability.
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
- [ ] **Categorized voxel viewer ("see the voxels": occupied / ground / free).**
      Preferred: **RViz2** — add an `rviz/` config that shows the categorized
      `voxel_mapper_node` layers + `/map` + `/points` + `/pose`, each toggled by its
      own checkbox (this is the ROS-native version of the Technical Spec's PyQt6/PyVista
      GUI, for far less effort). Fallback: the custom PyVista/PyQt6 GUI from
      `autonomous_rc_car/README.md` §6 only if an embeddable non-ROS viewer is wanted.
      *(Today's `pc_viewer.py` G-preview shows only the 2D grid: green=free / amber=
      inflation / red=obstacle — not per-category 3D voxel layers.)*

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
