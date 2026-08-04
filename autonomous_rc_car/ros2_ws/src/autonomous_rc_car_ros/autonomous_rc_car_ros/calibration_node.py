"""calibration_node: measure this car's driving constants against the phone.

Press ``c`` in the control console. This node then drives a short scripted
sequence and watches the iPhone's ARKit pose to work out what a motor unit
actually does on this chassis:

  1. preflight     -- pose fresh, car link up
  2. deadband      -- ramp up until it actually moves, forwards and spinning
  3. turn sign     -- which way does (left=+s, right=-s) rotate theta
  4. angular gain  -- rad/s per motor unit, from spins at three speeds
  5. straight run  -- m/s per unit, straightness trim, and command latency
  6. verification  -- one more straight run with the trim applied

Results are written to config/calibration.yaml and announced on
/calibration_result, so the driver and controller pick them up without a restart.

It measures against **/pose** (raw ARKit VIO) rather than /pose_corrected: ICP
applies discrete jumps that would corrupt these short-move measurements.

Motor ownership: while running, this node publishes a latched /calibration_active
and drives through /drive_raw, which car_driver_node obeys instead of /drive.
Without that the motion controller's 10 Hz HOLD stream would fight every move.

SAFETY: this cannot be run with the wheels off the ground -- it measures real
motion. It needs roughly 2x2 m of clear floor and drives forward about 1.5 m.
Press h or SPACE in the console to abort at any point.

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
from nav.calibration import (
    Calibration,
    detect_deadband,
    fit_gain,
    latency_from_step,
    trim_from_drift,
)
from nav.frames import points_ros_to_arkit, quaternion_to_matrix, rotation_ros_to_arkit
from nav.localization import angle_diff, pose_to_2d

SAMPLE_HZ = 20.0
SETTLE = 0.5              # seconds of stillness between stages
RAMP_STEP = 2             # motor units per rung of the deadband ramp
RAMP_DWELL = 0.6          # seconds per rung
MOVE_THRESHOLD = 0.03     # meters that count as "it moved"
TURN_MOVE_THRESHOLD = math.radians(5.0)
SPIN_TIME = 1.2
STRAIGHT_TIME = 1.2
MAX_UNIT = 70             # never command more than this while calibrating
RUNAWAY_DIST = 2.5        # meters from a stage's start before we call it a runaway
POSE_STALE = 0.5          # seconds without a pose before we abort
# The console disarms motion before triggering, so that disarm arrives just
# after this starts and looks like the operator hitting stop. Ignore
# /motion_enable briefly after launch.
ENABLE_GRACE = 1.5


class Aborted(Exception):
    """Raised inside the worker thread to unwind to a safe stop."""


class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        self.declare_parameter('straight_speeds', [10, 25, 40])  # above deadband
        self._speed_offsets = list(
            self.get_parameter('straight_speeds').get_parameter_value().integer_array_value
        ) or [10, 25, 40]

        self._lock = threading.Lock()
        self._pose = None            # (stamp, x, z, theta)
        self._abort = threading.Event()
        self._worker = None
        self._started_at = 0.0

        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)

        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self.create_subscription(Empty, '/calibrate_trigger', self._on_trigger, 10)
        self.create_subscription(Bool, '/motion_enable', self._on_enable, latched)
        self.create_subscription(String, '/car_link', self._on_link, latched)

        self._drive_pub = self.create_publisher(String, '/drive_raw', 10)
        self._active_pub = self.create_publisher(Bool, '/calibration_active', latched)
        self._status_pub = self.create_publisher(String, '/calibration_status', latched)
        self._result_pub = self.create_publisher(String, '/calibration_result', latched)

        self._link_state = 'unknown'
        self._active_pub.publish(Bool(data=False))
        self._status('idle -- press c to calibrate')
        self.get_logger().info(
            'calibration_node up: press c in the console. Needs ~2x2 m of clear '
            'floor; the car MUST be on the ground.')

    # --- inputs -----------------------------------------------------------

    def _on_pose(self, msg: PoseStamped):
        p, o = msg.pose.position, msg.pose.orientation
        pos = points_ros_to_arkit(np.array([[p.x, p.y, p.z]], dtype=np.float32))[0]
        rot = rotation_ros_to_arkit(quaternion_to_matrix(o.x, o.y, o.z, o.w))
        mat = np.eye(4)
        mat[:3, :3] = rot
        mat[:3, 3] = pos
        x, z, theta = pose_to_2d(mat)
        with self._lock:
            self._pose = (time.monotonic(), x, z, theta)

    def _on_link(self, msg: String):
        self._link_state = msg.data

    def _on_enable(self, msg: Bool):
        # h / SPACE in the console means stop, and that includes stopping this.
        if not msg.data and self._running:
            if time.monotonic() - self._started_at < ENABLE_GRACE:
                return          # the console's own disarm, not an abort
            self.get_logger().warn('calibration aborted by the operator')
            self._abort.set()

    def _on_trigger(self, _msg: Empty):
        if self._running:
            self.get_logger().warn('calibration already running')
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

    def _drive(self, left, right):
        left = max(-MAX_UNIT, min(MAX_UNIT, int(left)))
        right = max(-MAX_UNIT, min(MAX_UNIT, int(right)))
        # The sequence runs on a worker thread, so a Ctrl-C can tear the rclpy
        # context down underneath it; publishing then raises and kills the
        # process with a traceback.
        if not rclpy.ok():
            return
        try:
            self._drive_pub.publish(String(data=f'L{left}R{right}'))
        except Exception:   # noqa: BLE001 - shutting down
            pass

    def _stop(self):
        self._drive(0, 0)

    def _sleep(self, seconds):
        """Sleep in small slices so an abort is felt immediately."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self._abort.is_set():
                raise Aborted()
            time.sleep(0.02)

    def _pose_now(self):
        with self._lock:
            pose = self._pose
        if pose is None:
            raise Aborted('no pose from the phone -- is it streaming?')
        stamp, x, z, theta = pose
        if time.monotonic() - stamp > POSE_STALE:
            raise Aborted('pose went stale -- phone tracking lost')
        return x, z, theta

    def _hold(self, left, right, duration):
        """Drive (left, right) for `duration`, sampling the pose throughout.

        Returns [(elapsed, x, z, theta)] starting at the moment of the command.
        Re-sends every tick: the firmware's 500 ms failsafe needs feeding.
        """
        start_x, start_z, _ = self._pose_now()
        samples = []
        started = time.monotonic()
        period = 1.0 / SAMPLE_HZ
        while True:
            elapsed = time.monotonic() - started
            if elapsed >= duration:
                break
            if self._abort.is_set():
                raise Aborted()
            self._drive(left, right)
            x, z, theta = self._pose_now()
            if math.hypot(x - start_x, z - start_z) > RUNAWAY_DIST:
                raise Aborted('runaway: the car travelled further than expected')
            samples.append((elapsed, x, z, theta))
            time.sleep(period)
        self._stop()
        return samples

    def _settle(self):
        self._stop()
        self._sleep(SETTLE)

    @staticmethod
    def _displacement(samples):
        _, x0, z0, _ = samples[0]
        _, x1, z1, _ = samples[-1]
        return math.hypot(x1 - x0, z1 - z0)

    @staticmethod
    def _rotation(samples):
        """Total signed rotation, accumulated so it survives the +-pi wrap."""
        total = 0.0
        for (_, _, _, a), (_, _, _, b) in zip(samples, samples[1:]):
            total += angle_diff(b, a)
        return total

    @staticmethod
    def _elapsed(samples):
        return samples[-1][0] - samples[0][0]

    # --- stages -----------------------------------------------------------

    def _preflight(self):
        self._status('preflight')
        self._pose_now()                       # raises if missing or stale
        if not self._link_state.startswith('connected'):
            raise Aborted(f'car link is {self._link_state!r} -- is the ESP32 powered?')

    def _ramp_until_it_moves(self, spin):
        """Step the speed up until the car reacts. Returns (samples, deadband)."""
        measurements = []
        for unit in range(RAMP_STEP, MAX_UNIT + 1, RAMP_STEP):
            self._status(f'deadband {"spin" if spin else "drive"}: trying {unit}')
            left, right = (unit, -unit) if spin else (unit, unit)
            samples = self._hold(left, right, RAMP_DWELL)
            moved = (abs(self._rotation(samples)) if spin
                     else self._displacement(samples))
            measurements.append((unit, moved))
            threshold = TURN_MOVE_THRESHOLD if spin else MOVE_THRESHOLD
            if moved >= threshold:
                break
            self._sleep(0.15)
        threshold = TURN_MOVE_THRESHOLD if spin else MOVE_THRESHOLD
        deadband = detect_deadband(measurements, threshold)
        if deadband is None:
            raise Aborted(
                f'the car never moved up to {MAX_UNIT} units -- check power, '
                'wiring and that the wheels are on the ground')
        return deadband

    def _measure_turn_sign(self, turn_deadband):
        self._status('turn sign')
        unit = min(turn_deadband + 20, MAX_UNIT)
        samples = self._hold(unit, -unit, SPIN_TIME)
        rotation = self._rotation(samples)
        if abs(rotation) < TURN_MOVE_THRESHOLD:
            raise Aborted('the car did not rotate -- cannot determine turn sign')
        return (1 if rotation > 0 else -1), abs(rotation) / self._elapsed(samples), unit

    def _measure_angular_gain(self, turn_deadband, seed):
        """Spin at three speeds; fit rad/s per motor unit."""
        measurements = [seed]
        for offset in self._speed_offsets:
            unit = min(turn_deadband + offset, MAX_UNIT)
            if any(abs(unit - u) < 1 for u, _ in measurements):
                continue
            self._status(f'angular gain: spinning at {unit}')
            self._settle()
            samples = self._hold(unit, -unit, SPIN_TIME)
            measurements.append((unit, abs(self._rotation(samples))
                                 / self._elapsed(samples)))
        gain, _ = fit_gain(measurements)
        return gain

    def _straight_run(self, unit, trim=0.0):
        """One forward run. Returns (speed, drift, distance, speed_trace)."""
        left = unit * (1.0 + trim)
        right = unit * (1.0 - trim)
        samples = self._hold(left, right, STRAIGHT_TIME)
        distance = self._displacement(samples)
        elapsed = self._elapsed(samples)
        drift = self._rotation(samples)

        trace = []
        for (t0, x0, z0, _), (t1, x1, z1, _) in zip(samples, samples[1:]):
            dt = t1 - t0
            trace.append((t1, math.hypot(x1 - x0, z1 - z0) / dt if dt > 0 else 0.0))
        return distance / elapsed if elapsed > 0 else 0.0, drift, distance, trace

    def _measure_straight(self, drive_deadband, turn_sign):
        """Linear gain, straightness trim and command latency in one pass."""
        measurements = []
        latency = None
        trim = 0.0
        for offset in self._speed_offsets:
            unit = min(drive_deadband + offset, MAX_UNIT)
            self._status(f'straight run at {unit}')
            self._settle()
            speed, drift, distance, trace = self._straight_run(unit)
            measurements.append((unit, speed))
            if latency is None:
                latency = latency_from_step(trace, command_time=0.0)
            if distance > 0.1:
                trim = trim_from_drift(drift, distance, turn_sign)
        gain, _ = fit_gain(measurements)
        return gain, trim, (latency or 0.0)

    def _verify(self, drive_deadband, trim):
        self._status('verifying the trim')
        self._settle()
        unit = min(drive_deadband + self._speed_offsets[-1], MAX_UNIT)
        _, drift, distance, _ = self._straight_run(unit, trim)
        if distance <= 0.1:
            return None
        return abs(drift) / distance      # residual rad per meter

    # --- the run ----------------------------------------------------------

    def _run(self):
        result = None
        try:
            self._active_pub.publish(Bool(data=True))
            self.get_logger().warn('CALIBRATING -- the car will move on its own')
            self._preflight()

            self._settle()
            drive_deadband = self._ramp_until_it_moves(spin=False)
            self._settle()
            turn_deadband = self._ramp_until_it_moves(spin=True)

            self._settle()
            turn_sign, spin_rate, spin_unit = self._measure_turn_sign(turn_deadband)
            angular_gain = self._measure_angular_gain(
                turn_deadband, seed=(spin_unit, spin_rate))

            linear_gain, trim, latency = self._measure_straight(
                drive_deadband, turn_sign)
            residual = self._verify(drive_deadband, trim)

            result = Calibration(
                turn_sign=turn_sign,
                drive_deadband=int(drive_deadband),
                turn_deadband=int(turn_deadband),
                linear_gain=float(linear_gain),
                angular_gain=float(angular_gain),
                straightness_trim=float(trim),
                command_latency=float(latency),
            )
            path = config.calibration_path()
            path.parent.mkdir(parents=True, exist_ok=True)
            result.save(path)
            config.reload_calibration()

            summary = (f'turn_sign={turn_sign} deadband={drive_deadband}/{turn_deadband} '
                       f'linear={linear_gain:.4f}m/s/unit '
                       f'angular={angular_gain:.4f}rad/s/unit '
                       f'trim={trim:+.3f} latency={latency:.2f}s')
            if residual is not None:
                summary += f' residual={residual:.3f}rad/m'
            self.get_logger().info(f'calibration done -> {path}')
            self.get_logger().info(summary)
            self._status(f'done: {summary}')
            self._result_pub.publish(String(data=str(path)))

        except Aborted as exc:
            reason = str(exc) or 'aborted'
            self.get_logger().warn(f'calibration stopped: {reason}')
            self._status(f'ABORTED: {reason}')
        except Exception as exc:                      # noqa: BLE001 - must not leave the car driving
            self.get_logger().error(f'calibration failed: {exc}')
            self._status(f'FAILED: {exc}')
        finally:
            self._stop()
            self._stop()                              # belt and braces
            self._active_pub.publish(Bool(data=False))
        return result


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationNode()
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
