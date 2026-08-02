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

from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String

from nav import config
from nav.controller import WaypointFollower
from nav.frames import points_ros_to_arkit, quaternion_to_matrix, rotation_ros_to_arkit
from nav.localization import pose_to_2d


class MotionControllerNode(Node):
    def __init__(self):
        super().__init__('motion_controller_node')
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('start_enabled', False)   # testing mode: HOLD by default
        rate = self.get_parameter('rate_hz').get_parameter_value().double_value
        self._enabled = self.get_parameter('start_enabled').get_parameter_value().bool_value

        self._pose2d = None      # (x, z, theta) in nav world
        self._follower = None

        path_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Path, '/cmd_path', self._on_path, path_qos)
        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self._drive_pub = self.create_publisher(String, '/drive', 10)
        self._intended_pub = self.create_publisher(String, '/drive_intended', 10)

        # Latched so the controller picks up the last GO/HOLD even if it (re)starts
        # after the enable button.
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/motion_enable', self._on_enable, latched)
        # TURN_SIGN comes from the calibration file; re-read it when calibration
        # writes a fresh one so the car turns the right way without a restart.
        self.create_subscription(String, '/calibration_result',
                                 self._on_calibration_result, latched)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            'motion_controller_node up: /cmd_path + /pose -> /drive '
            f'(motion {"ENABLED" if self._enabled else "HOLD"}; /drive_intended shows the plan)')

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

    def _on_enable(self, msg: Bool):
        if msg.data != self._enabled:
            self.get_logger().warn(
                'MOTION ENABLED -- car will follow the path' if msg.data
                else 'MOTION HOLD -- car stopped')
        self._enabled = msg.data

    def _on_calibration_result(self, msg: String):
        cal = config.reload_calibration()
        self.get_logger().info(
            f'reloaded calibration from {msg.data}: turn_sign {cal.turn_sign}')

    def _tick(self):
        left = right = 0
        if self._follower is not None and self._pose2d is not None and not self._follower.done:
            x, z, theta = self._pose2d
            left, right = self._follower.update(x, z, theta)
        # Always publish what the controller WOULD do (for display/preview)...
        self._intended_pub.publish(String(data=f'L{int(left)}R{int(right)}'))
        # ...but only actually drive when enabled; otherwise command a stop.
        if self._enabled:
            self._drive_pub.publish(String(data=f'L{int(left)}R{int(right)}'))
        else:
            self._drive_pub.publish(String(data='L0R0'))


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
