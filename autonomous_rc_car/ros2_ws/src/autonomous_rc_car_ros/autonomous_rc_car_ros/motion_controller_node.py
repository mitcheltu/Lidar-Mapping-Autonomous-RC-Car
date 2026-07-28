"""Motion controller node (STUB).

Intended behavior:
    Subscribes:
        /cmd_path  (nav_msgs/Path)           -- path to follow
        /pose      (geometry_msgs/PoseStamped) -- current pose
    Publishes:
        /drive     (std_msgs/String)         -- "L..R.." motor command

Implements pure-pursuit / waypoint following: uses ``nav.localization.pose_to_2d``
to get (x, z, theta), tracks the active waypoint from /cmd_path, and emits an
"L<left>R<right>" motor command string consumed by bridge_node -> ESP32.

NOTE: ROS2 Humble / build + run in WSL2. Not yet implemented.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from nav import localization  # noqa: F401  (used once implemented)


class MotionControllerNode(Node):
    """Follows a path with pure-pursuit and emits drive commands (stub)."""

    def __init__(self):
        super().__init__('motion_controller_node')
        self.get_logger().warn('TODO: motion_controller_node not yet implemented')

        self._path_sub = self.create_subscription(
            Path, '/cmd_path', self._on_path, 1
        )
        self._pose_sub = self.create_subscription(
            PoseStamped, '/pose', self._on_pose, 10
        )
        self._drive_pub = self.create_publisher(String, '/drive', 10)

    def _on_path(self, msg):
        # TODO: store waypoints for the follower.
        pass

    def _on_pose(self, msg):
        # TODO: pose_to_2d -> pure-pursuit -> publish "L..R.." on /drive.
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MotionControllerNode()
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
