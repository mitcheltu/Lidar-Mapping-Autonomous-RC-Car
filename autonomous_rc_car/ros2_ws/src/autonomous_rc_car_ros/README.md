# autonomous_rc_car_ros

ROS2 (Humble, `ament_python`) bringup for the autonomous RC car. It ingests the
iPhone LiDAR/pose/image TCP stream, republishes it onto ROS topics, and hosts
the mapping / planning / control / SLAM nodes. The heavy algorithms live in the
pip-installable `nav` library (`autonomous_rc_car/laptop_brain`); the ROS nodes
are thin wrappers around it.

> Only `bridge_node` is implemented. `voxel_mapper_node`, `frontier_planner_node`,
> `motion_controller_node` and `icp_slam_node` are **stubs** that warn on startup
> and wire up their topics but do no real work yet.

## Topic graph

- `bridge_node` (real): TCP `:9000` -> `/points` (PointCloud2), `/pose`
  (PoseStamped), `/image` (CompressedImage); subscribes `/drive` (String) and
  relays it back over the socket to the phone/ESP32.
- `voxel_mapper_node` (stub): `/points` + `/pose` -> `/map` (nav_msgs/OccupancyGrid)
  via `nav.mapping` (clean -> floor -> build_occupancy_grid -> inflate).
- `frontier_planner_node` (stub): `/map` -> `/cmd_path` (nav_msgs/Path) via
  `nav.frontier.choose_goal` + `nav.planner.astar` / `simplify_path`.
- `motion_controller_node` (stub): `/cmd_path` + `/pose` -> `/drive` (String,
  `"L..R.."`) via pure-pursuit over `nav.localization.pose_to_2d`.
- `icp_slam_node` (stub): `/points` + `/pose` -> `/pose_corrected` (PoseStamped)
  via KISS-ICP scan-to-map.

## Build & run (WSL2, ROS2 Humble sourced)

This package must be built and run under Linux/WSL2 — there is no ROS2 on
Windows. The files are authored on Windows only.

```bash
# in the ROS2 workspace (WSL2, ROS2 Humble sourced)
pip install -e ../../../laptop_brain          # install the nav library
cd autonomous_rc_car/ros2_ws
colcon build --packages-select autonomous_rc_car_ros
source install/setup.bash
ros2 launch autonomous_rc_car_ros bringup.launch.py
```

The `nav` library is a **pip dependency**, not a rosdep/ament package — install
it into the same Python environment before `colcon build`.
