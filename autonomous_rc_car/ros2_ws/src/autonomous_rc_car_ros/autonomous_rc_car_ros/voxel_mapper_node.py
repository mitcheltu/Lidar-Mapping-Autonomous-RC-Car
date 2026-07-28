"""voxel_mapper_node: streamed point cloud -> occupancy grid + voxel layers.

Accumulates the world-frame points published by ``bridge_node`` on ``/points``
and, on a timer, runs the ``nav`` mapping pipeline (clean -> floor estimate ->
height-banded occupancy grid -> robot-radius inflation) and publishes:

    /map              nav_msgs/OccupancyGrid       (unknown/free/inflation/occupied)
    /voxels/ground    visualization_msgs/MarkerArray  (drivable floor voxel cubes)
    /voxels/obstacle  visualization_msgs/MarkerArray  (obstacle voxel cubes)

The ``/voxels/*`` layers are true 3D voxels: the cloud is binned into cubes of
edge ``voxel_size`` and a cube is emitted when enough LiDAR hits land in it
(``nav.voxel.voxelize``), rendered as CUBE_LIST markers so RViz2 shows actual
cubes -- toggle each layer independently. All heavy lifting lives in the
pip-installed ``nav`` library; this node is a thin ROS2 wrapper.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from nav.frames import points_arkit_to_ros, points_ros_to_arkit
from nav.mapping import build_occupancy_grid, clean_cloud, estimate_floor_height, inflate
from nav.ros_export import grid_to_occupancy
from nav.voxel import voxelize


class VoxelMapperNode(Node):
    def __init__(self):
        super().__init__('voxel_mapper_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rebuild_period', 1.5)   # seconds between map rebuilds
        self.declare_parameter('min_points', 500)       # below this, skip a rebuild
        self.declare_parameter('max_points', 400000)    # accumulation cap (most recent kept)
        self.declare_parameter('cell_size', 0.05)       # meters per 2D grid cell
        self.declare_parameter('robot_radius', 0.14)    # inflation radius (meters)
        self.declare_parameter('voxel_size', 0.03)      # 3D visualization voxel edge (m)
        self.declare_parameter('min_points_obstacle', 2)  # LiDAR hits to mark an obstacle voxel

        self._frame_id = self._p('frame_id').string_value
        self._min_points = self._p('min_points').integer_value
        self._max_points = self._p('max_points').integer_value
        self._cell_size = self._p('cell_size').double_value
        self._robot_radius = self._p('robot_radius').double_value
        self._voxel_size = self._p('voxel_size').double_value
        self._min_pts_obstacle = self._p('min_points_obstacle').integer_value
        period = self._p('rebuild_period').double_value

        self._store = np.zeros((0, 3), dtype=np.float32)

        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self._ground_pub = self.create_publisher(MarkerArray, '/voxels/ground', 1)
        self._obstacle_pub = self.create_publisher(MarkerArray, '/voxels/obstacle', 1)

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
        # /points arrive in the ROS z-up frame; nav works in ARKit y-up.
        xyz = points_ros_to_arkit(xyz)
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

        voxels = voxelize(cleaned, floor_y, voxel_size=self._voxel_size,
                          min_points_obstacle=self._min_pts_obstacle)
        # voxel centers are ARKit y-up; publish them in the ROS z-up frame.
        ground = points_arkit_to_ros(voxels['ground'])
        obstacle = points_arkit_to_ros(voxels['obstacle'])
        self._ground_pub.publish(self._cube_markers(
            ground, 'ground', ColorRGBA(r=0.15, g=0.75, b=0.25, a=0.9), header))
        self._obstacle_pub.publish(self._cube_markers(
            obstacle, 'obstacle', ColorRGBA(r=0.90, g=0.15, b=0.15, a=0.9), header))

    def _cube_markers(self, centers, ns, color, header):
        """One CUBE_LIST marker of edge voxel_size at each voxel center."""
        m = Marker()
        m.header = header
        m.ns = ns
        m.id = 0
        m.type = Marker.CUBE_LIST
        m.action = Marker.ADD
        m.scale.x = m.scale.y = m.scale.z = self._voxel_size
        m.color = color
        m.pose.orientation.w = 1.0
        m.points = [Point(x=float(c[0]), y=float(c[1]), z=float(c[2])) for c in centers]
        arr = MarkerArray()
        arr.markers = [m]
        return arr

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
