"""car_driver_node: /drive -> the ESP32 over WiFi.

The last link in the chain. Takes the ``"L<left>R<right>"`` commands the motion
controller publishes, applies this car's measured calibration (stiction deadband
and straightness trim -- see nav.calibration), and writes them as lines to the
ESP32's TCP server (esp32_car_wifi_tb6612.ino, port 9001). The phone is not
involved: it stays a pure perception sensor.

Motor ownership is explicit. While /calibration_active is true, calibration_node
owns the car through /drive_raw and /drive is ignored -- otherwise the motion
controller's 10 Hz stream of L0R0 during HOLD would fight every calibration move.
Commands on /drive_raw bypass the calibration mapping, because calibration has to
measure the raw hardware.

Publishes link state on /car_link. Reconnects on its own; commands a stop
whenever the socket drops.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import re
import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String

from nav import config

DRIVE_RE = re.compile(r'^L(-?\d+)R(-?\d+)$')


def parse_drive(text):
    """``"L60R-40"`` -> (60, -40). Returns None if it is not a drive command."""
    match = DRIVE_RE.match(text.strip())
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


class CarDriverNode(Node):
    def __init__(self):
        super().__init__('car_driver_node')
        self.declare_parameter('host', 'rccar.local')
        self.declare_parameter('port', 9001)
        self.declare_parameter('connect_timeout', 2.0)
        self.declare_parameter('reconnect_period', 2.0)

        self._host = self.get_parameter('host').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self._timeout = self.get_parameter(
            'connect_timeout').get_parameter_value().double_value
        reconnect = self.get_parameter(
            'reconnect_period').get_parameter_value().double_value

        self._sock = None
        self._calibrating = False
        self._scanning = False
        self._last_rtt = None
        self._warned = False

        self.create_subscription(String, '/drive', self._on_drive, 10)
        self.create_subscription(String, '/drive_raw', self._on_drive_raw, 10)

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, '/calibration_active',
                                 self._on_calibration_active, latched)
        self.create_subscription(Bool, '/scan_active',
                                 self._on_scan_active, latched)
        # nav.config caches the calibration per process, so a fresh result has to
        # be re-read here rather than waiting for a restart.
        self.create_subscription(String, '/calibration_result',
                                 self._on_calibration_result, latched)
        self._link_pub = self.create_publisher(String, '/car_link', latched)

        self.create_timer(reconnect, self._ensure_connected)
        self.get_logger().info(
            f'car_driver_node up: /drive -> {self._host}:{self._port} '
            f'(deadband {config.CALIBRATION.drive_deadband}, '
            f'trim {config.CALIBRATION.straightness_trim:+.3f})')

    # --- link ------------------------------------------------------------

    def _ensure_connected(self):
        if self._sock is not None:
            return
        try:
            sock = socket.create_connection((self._host, self._port), self._timeout)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(self._timeout)
            self._sock = sock
            self._warned = False
            self.get_logger().info(f'connected to the car at {self._host}:{self._port}')
            self._publish_link('connected')
        except OSError as exc:
            if not self._warned:      # once per outage, not once per retry
                self.get_logger().warn(
                    f'car not reachable at {self._host}:{self._port} ({exc}). Retrying...')
                self._warned = True
            self._publish_link('disconnected')

    def _drop(self, reason):
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
            self.get_logger().warn(f'car link lost: {reason}')
        self._publish_link('disconnected')

    def _publish_link(self, state):
        rtt = f' rtt={self._last_rtt * 1000:.0f}ms' if self._last_rtt else ''
        self._link_pub.publish(String(data=f'{state}{rtt}'))

    # --- commands --------------------------------------------------------

    @property
    def _raw_owner(self):
        """True while something other than the motion controller owns the car."""
        return self._calibrating or self._scanning

    def _on_calibration_active(self, msg: Bool):
        if msg.data != self._calibrating:
            self.get_logger().info(
                'calibration has the motors' if msg.data
                else 'calibration released the motors')
        self._calibrating = msg.data

    def _on_scan_active(self, msg: Bool):
        if msg.data != self._scanning:
            self.get_logger().info(
                'scan has the motors' if msg.data
                else 'scan released the motors')
        self._scanning = msg.data

    def _on_calibration_result(self, msg: String):
        cal = config.reload_calibration()
        self.get_logger().info(
            f'reloaded calibration from {msg.data}: deadband {cal.drive_deadband}, '
            f'trim {cal.straightness_trim:+.3f}')

    def _on_drive(self, msg: String):
        if self._raw_owner:
            return                       # calibration or scan owns the car
        command = parse_drive(msg.data)
        if command is None:
            self.get_logger().warn(f'ignoring malformed /drive: {msg.data!r}')
            return
        self._send(*config.CALIBRATION.apply(*command))

    def _on_drive_raw(self, msg: String):
        if not self._raw_owner:
            return                       # raw access only during calibration/scan
        command = parse_drive(msg.data)
        if command is None:
            self.get_logger().warn(f'ignoring malformed /drive_raw: {msg.data!r}')
            return
        self._send(*command)             # no mapping: measure the real hardware

    def _send(self, left, right):
        if self._sock is None:
            return
        sent_at = time.monotonic()
        try:
            self._sock.sendall(f'L{int(left)}R{int(right)}\n'.encode())
            if self._sock.recv(16):      # the firmware acks every command
                self._last_rtt = time.monotonic() - sent_at
        except OSError as exc:
            self._drop(str(exc))

    def stop_car(self):
        """Best-effort stop, used on shutdown."""
        if self._sock is not None:
            try:
                self._sock.sendall(b'S\n')
            except OSError:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = CarDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_car()                  # never leave the car driving
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
