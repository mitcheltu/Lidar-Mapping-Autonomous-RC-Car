"""scan_node: capture the surroundings on demand, instead of streaming forever.

Press ``s`` in the control console. This node then:

  1. stops the car and lets it settle
  2. puts the phone in SCAN mode, so it starts spending power on depth
  3. spins the car in place, watching /pose until it has turned a full circle
  4. stops
  5. puts the phone back in IDLE, so depth processing costs nothing again

The phone never decides to move and never decides when to scan -- the laptop owns
both. That keeps one brain in charge and lets the phone stay a dumb sensor.

Motor ownership is the same arbitration calibration uses: while scanning this
node publishes a latched /scan_active and drives through /drive_raw, which
car_driver_node obeys instead of /drive. Without that, the motion controller's
10 Hz stream of L0R0 during HOLD would fight every spin command.

Abort with h/SPACE in the console (it clears /motion_enable), or by a stale pose.

Build/run in WSL2 (ROS2 Humble) -- see autonomous_rc_car/ROS2_SETUP.md.
"""

import math
import threading
import time

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String

from nav import config
from nav.frames import points_ros_to_arkit, quaternion_to_matrix, rotation_ros_to_arkit
from nav.localization import pose_to_2d
from nav.scan import RotationTracker
from nav.stream_protocol import MODE_DRIVE, MODE_IDLE, MODE_SCAN

SETTLE = 0.6              # seconds of stillness before and after the spin
# Starting a scan from the console disarms motion first and triggers second, so
# that disarm lands just after the worker starts and looks exactly like the
# operator hitting stop. Ignore /motion_enable for a moment after launch.
ENABLE_GRACE = 1.5
POSE_STALE = 0.5          # seconds without a pose before we give up
FULL_TURN = 2.0 * math.pi
SPIN_TIMEOUT = 40.0       # a full circle should take a few seconds, not this
SAMPLE_PERIOD = 0.05


class Aborted(Exception):
    """Raised inside the worker thread to unwind to a safe stop."""


class ScanNode(Node):
    def __init__(self):
        super().__init__('scan_node')
        self.declare_parameter('spin_speed', float(config.SPIN_SPEED))
        self.declare_parameter('turn_fraction', 1.0)   # 1.0 = a full circle
        self._spin_speed = int(
            self.get_parameter('spin_speed').get_parameter_value().double_value)
        self._turn_fraction = self.get_parameter(
            'turn_fraction').get_parameter_value().double_value

        self._lock = threading.Lock()
        self._pose = None            # (stamp, theta)
        self._abort = threading.Event()
        self._worker = None
        self._started_at = 0.0

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self.create_subscription(Empty, '/scan_trigger', self._on_trigger, 10)
        self.create_subscription(Bool, '/motion_enable', self._on_enable, latched)

        self._drive_pub = self.create_publisher(String, '/drive_raw', 10)
        self._active_pub = self.create_publisher(Bool, '/scan_active', latched)
        self._mode_pub = self.create_publisher(String, '/sensor_mode', latched)
        self._status_pub = self.create_publisher(String, '/scan_status', latched)

        self._active_pub.publish(Bool(data=False))
        self._set_mode(MODE_IDLE)     # depth off until someone asks for it
        self._status('idle -- press s to scan')
        self.get_logger().info(
            'scan_node up: press s to spin and capture the surroundings '
            f'(spin_speed={self._spin_speed})')

    # --- inputs -----------------------------------------------------------

    def _on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        pos = points_ros_to_arkit(np.array([[p.x, p.y, p.z]], dtype=np.float32))[0]
        rot = rotation_ros_to_arkit(quaternion_to_matrix(o.x, o.y, o.z, o.w))
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = pos
        _, _, theta = pose_to_2d(mat)
        with self._lock:
            self._pose = (time.monotonic(), theta)

    def _on_enable(self, msg: Bool):
        if self._running:
            if time.monotonic() - self._started_at < ENABLE_GRACE:
                return          # the console's own disarm, not an abort
            if not msg.data:
                self.get_logger().warn('scan aborted by the operator')
                self._abort.set()
            return
        # Sole owner of /sensor_mode: two publishers on a latched topic would
        # race, so the driving/idle switch lives here rather than in the motion
        # controller. Armed means moving, which is when depth is wanted.
        self._set_mode(MODE_DRIVE if msg.data else MODE_IDLE)

    def _on_trigger(self, _msg: Empty):
        if self._running:
            self.get_logger().warn('scan already running')
            return
        self._abort.clear()
        self._started_at = time.monotonic()
        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()

    @property
    def _running(self):
        return self._worker is not None and self._worker.is_alive()

    # --- primitives -------------------------------------------------------

    def _status(self, text):
        if not rclpy.ok():
            return
        try:
            self._status_pub.publish(String(data=text))
        except Exception:   # noqa: BLE001 - shutting down
            pass

    def _set_mode(self, mode):
        if not rclpy.ok():
            return
        try:
            self._mode_pub.publish(String(data=mode))
        except Exception:   # noqa: BLE001 - shutting down
            pass

    def _drive(self, left, right):
        # The spin runs on a worker thread, so a Ctrl-C can tear the rclpy
        # context down underneath it. Publishing then raises and kills the
        # process with a traceback; check and swallow instead.
        if not rclpy.ok():
            return
        try:
            self._drive_pub.publish(String(data=f'L{int(left)}R{int(right)}'))
        except Exception:   # noqa: BLE001 - shutting down; nothing useful to do
            pass

    def _stop(self):
        self._drive(0, 0)

    def _sleep(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._abort.is_set():
                raise Aborted()
            time.sleep(0.02)

    def _theta_now(self):
        with self._lock:
            pose = self._pose
        if pose is None:
            raise Aborted('no pose from the phone -- is it streaming?')
        stamp, theta = pose
        if time.monotonic() - stamp > POSE_STALE:
            raise Aborted('pose went stale -- phone tracking lost')
        return theta

    # --- the run ----------------------------------------------------------

    def _spin_full_circle(self):
        """Rotate in place until a full turn has accumulated.

        The wrap-safe accumulation lives in nav.scan.RotationTracker, which is
        unit-tested; this loop only has to keep commanding the spin and watching
        for the operator, a timeout or a lost pose.
        """
        tracker = RotationTracker(start=self._theta_now(),
                                  target=FULL_TURN * self._turn_fraction)
        started = time.monotonic()
        sign = config.TURN_SIGN

        while not tracker.done:
            if self._abort.is_set():
                raise Aborted()
            if time.monotonic() - started > SPIN_TIMEOUT:
                raise Aborted(
                    f'spin timed out after {tracker.degrees:.0f} of '
                    f'{tracker.target_degrees:.0f} degrees -- is the car turning?')
            self._drive(sign * self._spin_speed, -sign * self._spin_speed)
            time.sleep(SAMPLE_PERIOD)
            tracker.update(self._theta_now())
            self._status(f'scanning: {tracker.degrees:.0f} / '
                         f'{tracker.target_degrees:.0f} deg')
        self._stop()

    def _run(self):
        try:
            self._active_pub.publish(Bool(data=True))
            self.get_logger().warn('SCANNING -- the car will spin on the spot')

            self._status('preparing')
            self._stop()
            self._theta_now()          # fail early if there is no pose at all
            self._sleep(SETTLE)

            self._set_mode(MODE_SCAN)  # depth on
            self._sleep(0.4)           # let the phone act on it before we move

            self._spin_full_circle()

            self._stop()
            self._sleep(SETTLE)        # capture the last frames while still
            self._status('scan complete')
            self.get_logger().info('scan complete')

        except Aborted as exc:
            reason = str(exc) or 'aborted'
            self.get_logger().warn(f'scan stopped: {reason}')
            self._status(f'ABORTED: {reason}')
        except Exception as exc:       # noqa: BLE001 - never leave the car spinning
            self.get_logger().error(f'scan failed: {exc}')
            self._status(f'FAILED: {exc}')
        finally:
            self._stop()
            self._stop()
            self._set_mode(MODE_IDLE)  # depth off again, whatever happened
            self._active_pub.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = ScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._abort.set()
        node._stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
