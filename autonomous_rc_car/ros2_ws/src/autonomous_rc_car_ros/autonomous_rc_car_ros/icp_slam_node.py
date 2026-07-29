"""icp_slam_node: /points + /pose -> /pose_corrected.

Corrects slow ARKit VIO drift by aligning the recent scan to an accumulated map
with ICP (``nav.drift``). A running correction transform is refined on a timer and
applied to every incoming /pose, republished as /pose_corrected -- which the
mapper/controller can consume instead of the raw pose once drift matters. Works
entirely in the ROS frame (ICP is frame-agnostic), so no conversions here.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from nav.drift import icp_correction, transform_points, voxel_downsample
from nav.frames import matrix_to_quaternion, quaternion_to_matrix


class IcpSlamNode(Node):
    def __init__(self):
        super().__init__('icp_slam_node')
        self.declare_parameter('update_period', 2.0)    # seconds between ICP refines
        self.declare_parameter('map_voxel', 0.05)       # map/scan downsample (m)
        self.declare_parameter('max_corr_dist', 0.15)   # ICP correspondence distance (m)
        self.declare_parameter('min_scan_points', 300)  # need this many to register
        self.declare_parameter('min_map_points', 800)   # seed the map until this many
        self.declare_parameter('max_map_points', 200000)
        self.declare_parameter('max_jump', 0.5)         # reject corrections bigger than this (m)

        self._map_voxel = self._p('map_voxel').double_value
        self._max_corr = self._p('max_corr_dist').double_value
        self._min_scan = self._p('min_scan_points').integer_value
        self._min_map = self._p('min_map_points').integer_value
        self._max_map = self._p('max_map_points').integer_value
        self._max_jump = self._p('max_jump').double_value

        self._map = np.zeros((0, 3), dtype=np.float32)
        self._scan = np.zeros((0, 3), dtype=np.float32)
        self._corr = np.eye(4)

        self.create_subscription(PointCloud2, '/points', self._on_points, 10)
        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self._pub = self.create_publisher(PoseStamped, '/pose_corrected', 10)
        self.create_timer(self._p('update_period').double_value, self._refine)

        self.get_logger().info('icp_slam_node up: /points + /pose -> /pose_corrected')

    def _p(self, name):
        return self.get_parameter(name).get_parameter_value()

    def _on_points(self, msg: PointCloud2):
        pts = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)
        if xyz.size:
            self._scan = np.vstack([self._scan, xyz])[-120000:]

    def _on_pose(self, msg: PoseStamped):
        self._pub.publish(self._corrected(msg))

    def _corrected(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        pos = self._corr[:3, :3] @ np.array([p.x, p.y, p.z]) + self._corr[:3, 3]
        rot = self._corr[:3, :3] @ quaternion_to_matrix(o.x, o.y, o.z, o.w)
        qx, qy, qz, qw = matrix_to_quaternion(rot)
        out = PoseStamped()
        out.header = msg.header
        out.pose.position.x = float(pos[0])
        out.pose.position.y = float(pos[1])
        out.pose.position.z = float(pos[2])
        out.pose.orientation.x = float(qx)
        out.pose.orientation.y = float(qy)
        out.pose.orientation.z = float(qz)
        out.pose.orientation.w = float(qw)
        return out

    def _refine(self):
        if self._scan.shape[0] < self._min_scan:
            return
        scan = voxel_downsample(self._scan, self._map_voxel)

        if self._map.shape[0] < self._min_map:
            self._merge(scan)
            self._scan = np.zeros((0, 3), dtype=np.float32)
            return

        T = icp_correction(scan, self._map, self._max_corr, init=self._corr)
        jump = float(np.linalg.norm(T[:3, 3] - self._corr[:3, 3]))
        if jump <= self._max_jump:
            self._corr = T
            self.get_logger().info(f'ICP correction updated (delta={jump:.3f} m)')
        else:
            self.get_logger().warn(f'rejected large ICP jump ({jump:.2f} m)')
        self._merge(scan)
        self._scan = np.zeros((0, 3), dtype=np.float32)

    def _merge(self, scan):
        corrected = transform_points(scan, self._corr)
        self._map = voxel_downsample(np.vstack([self._map, corrected]), self._map_voxel)
        if self._map.shape[0] > self._max_map:
            self._map = self._map[-self._max_map:]


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
