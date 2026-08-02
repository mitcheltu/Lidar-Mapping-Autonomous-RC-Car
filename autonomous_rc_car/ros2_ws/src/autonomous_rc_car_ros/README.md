# autonomous_rc_car_ros

ROS2 (Humble, `ament_python`) bringup for the autonomous RC car. It ingests the
iPhone LiDAR/pose/image TCP stream, republishes it onto ROS topics, and hosts
the mapping / planning / control / SLAM nodes. The heavy algorithms live in the
pip-installable `nav` library (`autonomous_rc_car/laptop_brain`); the ROS nodes
are thin wrappers around it.

> All nine nodes are implemented: `bridge_node`, `voxel_mapper_node`,
> `frontier_planner_node`, `motion_controller_node`, `icp_slam_node`,
> `motion_enable_node` (control console), `rerun_viz_node` (visualizer),
> `car_driver_node` (ESP32 link) and `calibration_node`.

## Topic graph

- `bridge_node` (real): TCP `:9000` -> `/points` (PointCloud2), `/pose`
  (PoseStamped), `/image` (CompressedImage); subscribes `/drive` (String) and
  relays it back over the socket to the phone/ESP32.
- `voxel_mapper_node` (real): incremental **log-odds voxel grid** with ray carving
  (`nav.voxel_grid`) — integrates each `/points` batch from the `/pose` sensor
  origin, carving voxels it sees through. Publishes `/map` (nav_msgs/OccupancyGrid)
  plus 3D voxel cube layers `/voxels/ground` and `/voxels/obstacle`
  (visualization_msgs/MarkerArray, CUBE_LIST). Params: `rebuild_period`,
  `cell_size`, `robot_radius`, `voxel_size`, `max_range`, `max_rays`, `min_voxels`.
- `motion_controller_node` (real): `/cmd_path` + `/pose` -> `/drive` (std_msgs/String
  `"L..R.."`). Turn-then-drive `nav.controller.WaypointFollower`; stops (`L0R0`) when
  idle or the path is complete.
- `frontier_planner_node` (real): `/map` + `/pose` -> `/cmd_path` (nav_msgs/Path).
  Rebuilds the nav grid, locates the car cell, picks the nearest reachable frontier
  (`nav.frontier.choose_goal`) and plans an A* + line-of-sight path
  (`nav.planner.astar` / `simplify_path`). Empty path = exploration complete.
- `icp_slam_node` (real): `/points` + `/pose` -> `/pose_corrected` (PoseStamped).
  Aligns the recent scan to an accumulated map with Open3D ICP (`nav.drift`),
  refines a running drift correction, and applies it to the pose (large jumps
  rejected).
- `car_driver_node` (real): `/drive` -> the ESP32 over TCP (`host`:`port`, default
  `rccar.local`:9001). Applies the measured calibration (deadband, straightness
  trim) from `nav.config`, reconnects on its own, publishes `/car_link`. Obeys
  `/drive_raw` instead of `/drive` while `/calibration_active` is true, so exactly
  one thing owns the motors at a time.
- `calibration_node` (real): on `/calibrate_trigger` (the console's `c`), drives a
  scripted sequence and measures it against `/pose` to produce
  `config/calibration.yaml` -- turn sign, deadband, linear/angular gain,
  straightness trim, command latency. Publishes `/calibration_status`,
  `/calibration_active` and `/calibration_result`. Aborts on `/motion_enable` false.

## Build & run (WSL2, ROS2 Humble sourced)

This package must be built and run under Linux/WSL2 — there is no ROS2 on
Windows. The files are authored on Windows only.

Once the `nav` library is installed (`pip install -e ../../../laptop_brain`), the
whole stack boots with one command from `autonomous_rc_car/`:

```bash
./run.sh            # build if needed + graph + Rerun + control console
./run.sh --help     # flags
```

By hand:

```bash
cd autonomous_rc_car/ros2_ws
colcon build --packages-select autonomous_rc_car_ros
source install/setup.bash
ros2 launch autonomous_rc_car_ros bringup.launch.py
# args: viz:=true|false  connect_addr:=host:9876  continuous:=  start_enabled:=
ros2 run autonomous_rc_car_ros motion_enable_node   # separate terminal
```

`bringup.launch.py` starts eight nodes; `motion_enable_node` is deliberately not one
of them — it calls `tty.setraw` on stdin, and a launched process gets a pipe rather
than a TTY. `run.sh` therefore runs it in the foreground of your terminal.

The `nav` library is a **pip dependency**, not a rosdep/ament package — install
it into the same Python environment before `colcon build`.

## See the voxels in RViz2

RViz2 gives the "occupied / ground / free" toggling from the Technical Spec for
free. It ships with the ROS2 **desktop** install; on a `ros-base` install add it:

```bash
sudo apt install -y ros-humble-rviz2
rviz2
```

With `bridge_node` + `voxel_mapper_node` running and the phone streaming, set
**Fixed Frame = `map`**, then **Add** these displays (each has its own visibility
checkbox):

| Display | Topic | Notes |
|---|---|---|
| Map | `/map` | 2D costmap (free/occupied/unknown) the planner drives on |
| **MarkerArray** | `/voxels/ground` | green voxel **cubes** (drivable floor) |
| **MarkerArray** | `/voxels/obstacle` | red voxel **cubes** (obstacles) |
| PointCloud2 | `/points` | raw cloud, white / RGB (context) |
| Pose | `/pose` | phone position + heading |

The `/voxels/*` layers are actual cubes (edge `voxel_size`, default 0.03 m), not
points — toggle `/voxels/ground` vs `/voxels/obstacle` to inspect the height-band
classification; `/map` is the 2D grid the planner uses.
(A saved `.rviz` config can be added later once verified against a live build.)
