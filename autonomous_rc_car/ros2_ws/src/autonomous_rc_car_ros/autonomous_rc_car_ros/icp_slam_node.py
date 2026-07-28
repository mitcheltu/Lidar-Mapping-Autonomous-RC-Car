"""ICP SLAM node (STUB).

Intended behavior:
    Subscribes:
        /points          (sensor_msgs/PointCloud2)   -- LiDAR cloud
        /pose            (geometry_msgs/PoseStamped)  -- raw ARKit pose (prior)
    Publishes:
        /pose_corrected  (geometry_msgs/PoseStamped)  -- refined pose

Runs KISS-ICP style scan-to-map registration, using the incoming ARKit pose as
the motion prior and refining it against the accumulated map, to reduce drift.

NOTE: ROS2 Humble / build + run in WSL2. Not yet implemented.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2


class IcpSlamNode(Node):
    """Scan-to-map ICP pose correction (stub)."""

    def __init__(self):
        super().__init__('icp_slam_node')
        self.get_logger().warn('TODO: icp_slam_node not yet implemented')

        self._points_sub = self.create_subscription(
            PointCloud2, '/points', self._on_points, 10
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, '/pose', self._on_pose, 10
        )
        self._pose_pub = self.create_publisher(
            PoseStamped, '/pose_corrected', 10
        )

    def _on_points(self, msg):
        # TODO: register scan against map (KISS-ICP), publish /pose_corrected.
        pass

    def _on_pose(self, msg):
        # TODO: use as motion prior for ICP.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = IcpSlamNode()
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
