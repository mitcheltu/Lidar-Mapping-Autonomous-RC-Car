"""Voxel mapper node (STUB).

Intended behavior:
    Subscribes:
        /points  (sensor_msgs/PointCloud2)  -- decoded iPhone LiDAR cloud
        /pose    (geometry_msgs/PoseStamped) -- camera-to-world pose
    Publishes:
        /map     (nav_msgs/OccupancyGrid)   -- inflated 2D occupancy grid

Wraps ``nav.mapping``: clean_cloud -> estimate_floor_height ->
build_occupancy_grid(cell_size=0.05) -> inflate(robot_radius=0.12), then
converts the resulting ``nav.grid.OccupancyGrid`` into nav_msgs/OccupancyGrid.

NOTE: ROS2 Humble / build + run in WSL2. Not yet implemented.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2

from nav import mapping  # noqa: F401  (used once implemented)


class VoxelMapperNode(Node):
    """Builds an inflated 2D occupancy grid from LiDAR + pose (stub)."""

    def __init__(self):
        super().__init__('voxel_mapper_node')
        self.get_logger().warn('TODO: voxel_mapper_node not yet implemented')

        self._points_sub = self.create_subscription(
            PointCloud2, '/points', self._on_points, 10
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, '/pose', self._on_pose, 10
        )
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 1)

    def _on_points(self, msg):
        # TODO: accumulate cloud, run nav.mapping pipeline, publish /map.
        pass

    def _on_pose(self, msg):
        # TODO: cache latest pose for cloud transform / car cell.
        pass


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
