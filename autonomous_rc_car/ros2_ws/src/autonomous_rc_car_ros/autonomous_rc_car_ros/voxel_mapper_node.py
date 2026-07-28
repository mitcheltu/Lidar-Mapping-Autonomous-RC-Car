"""voxel_mapper_node: streamed point cloud -> occupancy grid + voxel layers.

Maintains an **incremental log-odds voxel grid** (``nav.voxel_grid.VoxelGrid``):
each /points batch is integrated from the current sensor origin (latest /pose),
casting rays that add occupied evidence at the hit and free evidence along the
way -- so stale/moved obstacles the sensor now sees through are **carved away**.
This is O(new points) per frame, not a re-voxelization of the whole cloud.

On a timer it reads the occupied voxels and publishes:

    /map              nav_msgs/OccupancyGrid          (unknown/free/inflation/occupied)
    /voxels/ground    visualization_msgs/MarkerArray  (drivable floor voxel cubes)
    /voxels/obstacle  visualization_msgs/MarkerArray  (obstacle voxel cubes)

All heavy lifting lives in the pip-installed ``nav`` library; this node is a thin
ROS2 wrapper. Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import ColorRGBA, Header
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray

from nav.frames import points_arkit_to_ros, points_ros_to_arkit
from nav.mapping import build_occupancy_grid, estimate_floor_height, inflate
from nav.ros_export import categorize_points, grid_to_occupancy
from nav.voxel_grid import VoxelGrid


class VoxelMapperNode(Node):
    def __init__(self):
        super().__init__('voxel_mapper_node')

        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('rebuild_period', 1.5)   # seconds between map rebuilds
        self.declare_parameter('min_voxels', 150)       # below this, skip a rebuild
        self.declare_parameter('cell_size', 0.05)       # meters per 2D grid cell
        self.declare_parameter('robot_radius', 0.14)    # inflation radius (meters)
        self.declare_parameter('voxel_size', 0.03)      # 3D voxel edge (m)
        self.declare_parameter('max_range', 6.0)        # ignore hits farther than this (m)
        self.declare_parameter('max_rays', 2000)        # carved rays per batch (bounded cost)

        self._frame_id = self._p('frame_id').string_value
        self._min_voxels = self._p('min_voxels').integer_value
        self._cell_size = self._p('cell_size').double_value
        self._robot_radius = self._p('robot_radius').double_value
        self._voxel_size = self._p('voxel_size').double_value
        self._max_rays = self._p('max_rays').integer_value
        period = self._p('rebuild_period').double_value

        self._voxels = VoxelGrid(voxel_size=self._voxel_size,
                                 max_range=self._p('max_range').double_value)
        self._origin = None   # sensor origin (ARKit world), from latest /pose

        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self._ground_pub = self.create_publisher(MarkerArray, '/voxels/ground', 1)
        self._obstacle_pub = self.create_publisher(MarkerArray, '/voxels/obstacle', 1)

        self.create_subscription(PointCloud2, '/points', self._on_points, 10)
        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self.create_timer(period, self._rebuild)

        self.get_logger().info(
            f'voxel_mapper_node up: incremental log-odds voxels '
            f'(voxel={self._voxel_size} m), /map every {period:.1f}s'
        )

    def _p(self, name):
        return self.get_parameter(name).get_parameter_value()

    def _on_pose(self, msg: PoseStamped):
        p = msg.pose.position
        self._origin = points_ros_to_arkit(
            np.array([[p.x, p.y, p.z]], dtype=np.float32))[0]

    def _on_points(self, msg: PointCloud2):
        if self._origin is None:
            return   # need a sensor origin to cast rays
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if xyz.size == 0:
            return
        xyz = points_ros_to_arkit(xyz)   # ROS z-up -> ARKit y-up for nav
        self._voxels.update(xyz, self._origin, max_rays=self._max_rays)

    def _rebuild(self):
        centers = self._voxels.occupied_centers()   # ARKit y-up voxel centers
        if centers.shape[0] < self._min_voxels:
            return
        try:
            floor_y = estimate_floor_height(centers)
            grid = inflate(
                build_occupancy_grid(centers, floor_y, cell_size=self._cell_size),
                robot_radius=self._robot_radius,
            )
        except ValueError as exc:
            self.get_logger().warn(f'map rebuild skipped: {exc}')
            return

        header = self._header()
        self._publish_map(grid, floor_y, header)

        layers = categorize_points(centers, floor_y)
        ground = points_arkit_to_ros(layers['ground'])
        obstacle = points_arkit_to_ros(layers['obstacle'])
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
