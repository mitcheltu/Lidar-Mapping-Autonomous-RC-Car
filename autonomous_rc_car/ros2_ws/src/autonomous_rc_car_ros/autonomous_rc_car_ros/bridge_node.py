"""Bridge node: iPhone TCP stream <-> ROS2.

This node opens a TCP server and accepts a single connection from the iPhone
capture app. The phone pushes a framed binary stream of LiDAR point clouds,
camera-to-world poses and JPEG images. Each incoming frame is decoded with the
shared ``nav.stream_protocol`` wire protocol and republished onto ROS topics:

    - point cloud (0x50) -> sensor_msgs/PointCloud2 on /points
    - pose        (0x4F) -> geometry_msgs/PoseStamped on /pose
    - image       (0x49) -> sensor_msgs/CompressedImage on /image

In the reverse direction the node subscribes to std_msgs/String on /drive and
forwards the latest DRIVE command back down the same socket as a framed
message (type 0x44 = 'D'), so the phone can relay it to the car's ESP32.

NOTE: This package targets ROS2 Humble and MUST be built and run in WSL2
(Windows has no ROS2). The files here are authored on Windows only.
"""

import socket
import struct
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Header, String

from sensor_msgs_py import point_cloud2

from nav import stream_protocol
from nav.frames import points_arkit_to_ros, rotation_arkit_to_ros
from nav.stream_protocol import (
    MESSAGE_TYPE_IMAGE,
    MESSAGE_TYPE_POINT_CLOUD,
    MESSAGE_TYPE_POSE,
    decode_points_payload,
    decode_pose_payload,
)

# 5-byte frame header: 1 byte message type + uint32 little-endian payload length.
HEADER_STRUCT = struct.Struct('<BI')

# DRIVE command frame type ('D'), sent back to the phone.
MESSAGE_TYPE_DRIVE = 0x44


def rotation_matrix_to_quaternion(m):
    """Convert a 3x3 rotation matrix to a quaternion (x, y, z, w).

    Uses the numerically stable branch method (Shepperd / "Baraff") so we do
    not need to pull in scipy or tf_transformations as a dependency.
    """
    m00, m01, m02 = m[0, 0], m[0, 1], m[0, 2]
    m10, m11, m12 = m[1, 0], m[1, 1], m[1, 2]
    m20, m21, m22 = m[2, 0], m[2, 1], m[2, 2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * np.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * np.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=np.float64)
    norm = np.linalg.norm(q)
    if norm > 0.0:
        q /= norm
    return q


class BridgeNode(Node):
    """Bridges the iPhone TCP capture stream to and from ROS2 topics."""

    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 9000)
        self.declare_parameter('frame_id', 'map')

        self._host = self.get_parameter('host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self._frame_id = (
            self.get_parameter('frame_id').get_parameter_value().string_value
        )

        # Publishers for the three decoded stream message types.
        self._points_pub = self.create_publisher(PointCloud2, '/points', 10)
        self._pose_pub = self.create_publisher(PoseStamped, '/pose', 10)
        self._image_pub = self.create_publisher(CompressedImage, '/image', 10)

        # Subscribe to drive commands to relay back down the socket.
        self._drive_sub = self.create_subscription(
            String, '/drive', self._on_drive, 10
        )
        # Sensor mode: tells the phone when depth is worth the battery. Latched,
        # so a phone that connects late still learns the current mode.
        mode_qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                              durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._mode_sub = self.create_subscription(
            String, '/sensor_mode', self._on_sensor_mode, mode_qos
        )

        # Shared connection handle, guarded by a lock (accessed by both the
        # network thread and the ROS executor thread).
        self._conn_lock = threading.Lock()
        self._conn = None
        self._latest_drive = None
        self._latest_mode = None    # re-sent when a phone (re)connects

        self._server_thread = threading.Thread(
            target=self._serve_forever, daemon=True
        )
        self._server_thread.start()

        self.get_logger().info(
            f'bridge_node listening on {self._host}:{self._port} '
            f'(frame_id="{self._frame_id}")'
        )

    # ------------------------------------------------------------------
    # TCP server
    # ------------------------------------------------------------------
    def _serve_forever(self):
        """Accept connections and process framed messages (background thread)."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self._host, self._port))
            srv.listen(1)
        except OSError as exc:
            self.get_logger().error(f'failed to bind/listen: {exc}')
            return

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
            except OSError:
                break
            self.get_logger().info(f'phone connected from {addr}')
            with self._conn_lock:
                self._conn = conn
            self._resend_mode()   # a late-joining phone still learns the mode
            try:
                self._handle_connection(conn)
            except (OSError, ConnectionError) as exc:
                self.get_logger().warn(f'connection error: {exc}')
            finally:
                with self._conn_lock:
                    if self._conn is conn:
                        self._conn = None
                try:
                    conn.close()
                except OSError:
                    pass
                self.get_logger().info('phone disconnected')

    def _handle_connection(self, conn):
        """Read framed messages off ``conn`` until it closes."""
        while rclpy.ok():
            header = recvall(conn, HEADER_STRUCT.size)
            if header is None:
                break
            mtype, length = HEADER_STRUCT.unpack(header)
            payload = b'' if length == 0 else recvall(conn, length)
            if payload is None:
                break

            # Validate/normalise via the shared protocol parser. This also
            # keeps us honest if the wire format ever gains framing checks.
            _mtype, payload = stream_protocol.parse_frame(header + payload)

            if mtype == MESSAGE_TYPE_POINT_CLOUD:
                self._publish_points(payload)
            elif mtype == MESSAGE_TYPE_POSE:
                self._publish_pose(payload)
            elif mtype == MESSAGE_TYPE_IMAGE:
                self._publish_image(payload)
            else:
                self.get_logger().warn(f'unknown message type 0x{mtype:02X}')

    # ------------------------------------------------------------------
    # Decoders -> publishers
    # ------------------------------------------------------------------
    def _publish_points(self, payload):
        xyz, _rgb = decode_points_payload(payload)
        xyz = np.asarray(xyz, dtype=np.float32)
        # ARKit (y-up) -> ROS (z-up) so the cloud is upright and shares one frame
        # with the pose, voxels and map.
        xyz = points_arkit_to_ros(xyz)

        header = self._make_header()
        # Points-only cloud keeps this simple and correct; colour can be added
        # later with create_cloud + a PointField layout.
        msg = point_cloud2.create_cloud_xyz32(header, xyz.tolist())
        self._points_pub.publish(msg)

    def _publish_pose(self, payload):
        vals = decode_pose_payload(payload)  # 16 col-major floats
        # Column-major -> row-major 4x4 camera-to-world (ARKit frame).
        mat = np.array(vals, dtype=np.float64).reshape(4, 4).T
        pos = points_arkit_to_ros(mat[:3, 3].reshape(1, 3))[0]
        rot_ros = rotation_arkit_to_ros(mat[:3, :3])

        msg = PoseStamped()
        msg.header = self._make_header()
        msg.pose.position.x = float(pos[0])
        msg.pose.position.y = float(pos[1])
        msg.pose.position.z = float(pos[2])

        qx, qy, qz, qw = rotation_matrix_to_quaternion(rot_ros)
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self._pose_pub.publish(msg)

    def _publish_image(self, payload):
        msg = CompressedImage()
        msg.header = self._make_header()
        msg.format = 'jpeg'
        msg.data = bytes(payload)
        self._image_pub.publish(msg)

    def _make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        return header

    # ------------------------------------------------------------------
    # Drive command relay (ROS -> socket)
    # ------------------------------------------------------------------
    def _on_drive(self, msg):
        cmd = msg.data
        self._latest_drive = cmd
        frame = stream_protocol.frame(MESSAGE_TYPE_DRIVE, cmd.encode('ascii'))
        with self._conn_lock:
            conn = self._conn
            if conn is None:
                self.get_logger().warn(
                    f'drive "{cmd}" dropped: no phone connected'
                )
                return
            try:
                conn.sendall(frame)
            except OSError as exc:
                self.get_logger().warn(f'failed to send drive command: {exc}')
                return
        self.get_logger().info(f'relayed drive command: {cmd}')

    # ------------------------------------------------------------------
    # Sensor mode relay (ROS -> socket)
    # ------------------------------------------------------------------
    def _on_sensor_mode(self, msg):
        """Tell the phone whether depth is wanted (IDLE / SCAN / DRIVE).

        An app that predates this message ignores the unknown frame type and
        keeps streaming exactly as before, so this is safe to publish always.
        """
        try:
            packet = stream_protocol.encode_mode_packet(msg.data)
        except ValueError as exc:
            self.get_logger().warn(f'refusing to send bad sensor mode: {exc}')
            return
        self._latest_mode = msg.data.strip().upper()
        if not self._send_mode(packet):
            # Not an error: the mode is remembered and re-sent on connect.
            self.get_logger().info(
                f'sensor mode {self._latest_mode} queued until the phone connects')
            return
        self.get_logger().info(f'sensor mode -> {self._latest_mode}')

    def _send_mode(self, packet):
        with self._conn_lock:
            conn = self._conn
            if conn is None:
                return False
            try:
                conn.sendall(packet)
            except OSError as exc:
                self.get_logger().warn(f'failed to send sensor mode: {exc}')
                return False
        return True

    def _resend_mode(self):
        """Push the current mode to a phone that just connected."""
        if self._latest_mode is None:
            return
        try:
            packet = stream_protocol.encode_mode_packet(self._latest_mode)
        except ValueError:
            return
        if self._send_mode(packet):
            self.get_logger().info(f'sent sensor mode {self._latest_mode} to the phone')


def recvall(conn, n):
    """Receive exactly ``n`` bytes, or ``None`` if the peer closes early."""
    chunks = bytearray()
    while len(chunks) < n:
        chunk = conn.recv(n - len(chunks))
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
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
