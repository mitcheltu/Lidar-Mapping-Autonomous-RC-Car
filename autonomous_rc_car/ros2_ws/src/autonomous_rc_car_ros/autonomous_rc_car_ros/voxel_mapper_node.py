"""voxel_mapper_node: streamed point cloud -> occupancy grid + voxel layers.

Accumulates the world-frame points published by ``bridge_node`` on ``/points``
and, on a timer, runs the ``nav`` mapping pipeline (clean -> floor estimate ->
height-banded occupancy grid -> robot-radius inflation) and publishes:

    /map              nav_msgs/OccupancyGrid   (unknown/free/inflation/occupied)
    /voxels/ground    sensor_msgs/PointCloud2  (drivable floor points)
    /voxels/obstacle  sensor_msgs/PointCloud2  (obstacle-height points)

The categorized ``/voxels/*`` layers are the "see the voxels" capability: toggle
each independently in RViz2 (plus /map for free space). All heavy lifting lives in
the pip-installed ``nav`` library; this node is a thin ROS2 wrapper.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2

from nav.mapping import build_occupancy_grid, clean_cloud, estimate_floor_height, inflate
from nav.ros_export import categorize_points, grid_to_occupancy


class VoxelMapperNode(Node):
    def __init__(self):
        super().__init__('voxel_mapper_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rebuild_period', 1.5)   # seconds between map rebuilds
        self.declare_parameter('min_points', 500)       # below this, skip a rebuild
        self.declare_parameter('max_points', 400000)    # accumulation cap (most recent kept)
        self.declare_parameter('cell_size', 0.05)       # meters per grid cell
        self.declare_parameter('robot_radius', 0.14)    # inflation radius (meters)

        self._frame_id = self._p('frame_id').string_value
        self._min_points = self._p('min_points').integer_value
        self._max_points = self._p('max_points').integer_value
        self._cell_size = self._p('cell_size').double_value
        self._robot_radius = self._p('robot_radius').double_value
        period = self._p('rebuild_period').double_value

        self._store = np.zeros((0, 3), dtype=np.float32)

        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self._ground_pub = self.create_publisher(PointCloud2, '/voxels/ground', 1)
        self._obstacle_pub = self.create_publisher(PointCloud2, '/voxels/obstacle', 1)

        self.create_subscription(PointCloud2, '/points', self._on_points, 10)
        self.create_timer(period, self._rebuild)

        self.get_logger().info(
            f'voxel_mapper_node up: rebuilding /map every {period:.1f}s '
            f'(cell={self._cell_size} m, robot_radius={self._robot_radius} m)'
        )

    def _p(self, name):
        return self.get_parameter(name).get_parameter_value()

    def _on_points(self, msg: PointCloud2):
        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if xyz.size == 0:
            return
        self._store = np.vstack([self._store, xyz])
        if self._store.shape[0] > self._max_points:
            self._store = self._store[-self._max_points:]

    def _rebuild(self):
        if self._store.shape[0] < self._min_points:
            return
        try:
            cleaned = clean_cloud(self._store, voxel_size=max(0.03, self._cell_size * 0.6))
            floor_y = estimate_floor_height(cleaned)
            grid = inflate(
                build_occupancy_grid(cleaned, floor_y, cell_size=self._cell_size),
                robot_radius=self._robot_radius,
            )
        except ValueError as exc:
            # e.g. no points in the navigation height band yet
            self.get_logger().warn(f'map rebuild skipped: {exc}')
            return

        header = self._header()
        self._publish_map(grid, floor_y, header)

        layers = categorize_points(cleaned, floor_y)
        self._ground_pub.publish(
            point_cloud2.create_cloud_xyz32(header, layers['ground'].tolist()))
        self._obstacle_pub.publish(
            point_cloud2.create_cloud_xyz32(header, layers['obstacle'].tolist()))

    def _publish_map(self, grid, floor_y, header):
        exp = grid_to_occupancy(grid)
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = exp.resolution
        msg.info.width = exp.width
        msg.info.height = exp.height
        origin = Pose()
        origin.position.x = exp.origin_x
        origin.position.y = exp.origin_y
        origin.position.z = float(floor_y)
        origin.orientation.w = 1.0
        msg.info.origin = origin
        msg.data = exp.data
        self._map_pub.publish(msg)

    def _header(self):
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = self._frame_id
        return h


def main(args=None):
    rclpy.init(args=args)
    node = VoxelMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
