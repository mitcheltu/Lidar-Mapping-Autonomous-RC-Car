"""motion_controller_node: /cmd_path + /pose -> /drive.

Follows the planned path with a turn-then-drive waypoint follower
(``nav.controller.WaypointFollower``). On a fixed timer it reconstructs the car's
2D pose (x, z, theta) from the latest /pose, computes an (left, right) motor
command, and publishes it as ``"L<left>R<right>"`` on /drive -- which bridge_node
relays over the socket to the phone -> ESP32. Publishes a stop (``L0R0``) when
there is no path, the pose is stale, or the path is complete.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from nav.controller import WaypointFollower
from nav.frames import points_ros_to_arkit, quaternion_to_matrix, rotation_ros_to_arkit
from nav.localization import pose_to_2d


class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        self.declare_parameter('rate_hz', 10.0)
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value

        self._pose2d = None      # (x, z, theta) in nav world
        self._follower = None

        self.create_subscription(Path, '/cmd_path', self._on_path, 1)
        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self._drive_pub = self.create_publisher(String, '/drive', 10)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info('motion_controller_node up: /cmd_path + /pose -> /drive')

    def _on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        pos = points_ros_to_arkit(np.array([[p.x, p.y, p.z]], dtype=np.float32))[0]
        rot = rotation_ros_to_arkit(quaternion_to_matrix(o.x, o.y, o.z, o.w))
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = pos
        self._pose2d = pose_to_2d(mat)   # (x, z, theta) in ARKit/nav world

    def _on_path(self, msg: Path):
        wps = []
        for ps in msg.poses:
            p = ps.pose.position
            w = points_ros_to_arkit(np.array([[p.x, p.y, p.z]], dtype=np.float32))[0]
            wps.append((float(w[0]), float(w[2])))   # (world x, world z)
        self._follower = WaypointFollower(waypoints=wps) if wps else None

    def _tick(self):
        if self._follower is None or self._pose2d is None or self._follower.done:
            self._publish(0, 0)
            return
        x, z, theta = self._pose2d
        left, right = self._follower.update(x, z, theta)
        self._publish(int(left), int(right))

    def _publish(self, left, right):
        msg = String()
        msg.data = f'L{left}R{right}'
        self._drive_pub.publish(msg)


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
