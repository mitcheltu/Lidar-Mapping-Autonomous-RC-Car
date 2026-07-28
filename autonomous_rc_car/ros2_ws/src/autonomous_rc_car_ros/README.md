# autonomous_rc_car_ros

ROS2 (Humble, `ament_python`) bringup for the autonomous RC car. It ingests the
iPhone LiDAR/pose/image TCP stream, republishes it onto ROS topics, and hosts
the mapping / planning / control / SLAM nodes. The heavy algorithms live in the
pip-installable `nav` library (`autonomous_rc_car/laptop_brain`); the ROS nodes
are thin wrappers around it.

> Implemented: `bridge_node`, `voxel_mapper_node`. Still **stubs** (warn on startup,
> wire up topics, no real work yet): `frontier_planner_node`, `motion_controller_node`,
> `icp_slam_node`.

## Topic graph

- `bridge_node` (real): TCP `:9000` -> `/points` (PointCloud2), `/pose`
  (PoseStamped), `/image` (CompressedImage); subscribes `/drive` (String) and
  relays it back over the socket to the phone/ESP32.
- `voxel_mapper_node` (real): accumulates `/points`, on a timer runs `nav.mapping`
  (clean -> floor -> build_occupancy_grid -> inflate) and publishes `/map`
  (nav_msgs/OccupancyGrid) plus categorized voxel layers `/voxels/ground` and
  `/voxels/obstacle` (PointCloud2). Params: `rebuild_period`, `cell_size`,
  `robot_radius`, `min_points`, `max_points`.
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

| Display | Topic | Suggested style |
|---|---|---|
| Map | `/map` | costmap scheme (free/occupied/unknown) |
| PointCloud2 | `/voxels/ground` | flat color, green |
| PointCloud2 | `/voxels/obstacle` | flat color, red |
| PointCloud2 | `/points` | raw cloud, white / RGB |
| Pose | `/pose` | phone position + heading |

Toggle `/voxels/ground` vs `/voxels/obstacle` to inspect the height-band
classification; `/map` shows the free/inflated/occupied grid the planner uses.
(A saved `.rviz` config can be added later once verified against a live build.)
