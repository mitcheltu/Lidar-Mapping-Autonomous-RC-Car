"""Drive calibration: measured constants for one physical car, and the math
that derives them from ARKit pose traces.

The ESP32 firmware is deliberately dumb -- it maps 0..100 straight onto PWM duty
and knows nothing about this car's stiction or its slightly mismatched motors.
Those live here, measured once against the phone's pose and stored in a YAML file
that is loaded at runtime, so re-calibrating never means reflashing or rebuilding.

Pure math and I/O only: no ROS, no sockets. `calibration_node` drives the car and
feeds the samples in; this module decides what they mean.
"""

from dataclasses import asdict, dataclass, field
from datetime import datetime

import yaml

# How hard to correct a curve, in trim units per (rad/m) of measured drift.
# Deliberately gentle -- an over-correcting trim oscillates.
TRIM_GAIN = 0.1
TRIM_LIMIT = 0.5          # never let trim swamp the command
STEP_FRACTION = 0.2       # "moving" = a fifth of steady speed, for latency


@dataclass
class Calibration:
    """One car's measured driving constants."""

    turn_sign: int = 1               # +1 if (left=+s, right=-s) increases theta
    drive_deadband: int = 0          # motor units below which it does not move
    turn_deadband: int = 0           # same, for rotation in place
    linear_gain: float = 0.0         # m/s per motor unit above the deadband
    angular_gain: float = 0.0        # rad/s per motor unit above the deadband
    straightness_trim: float = 0.0   # +ve strengthens left, weakens right
    command_latency: float = 0.0     # seconds from publish to visible motion
    calibrated_at: str = field(default="")

    # --- using it ---------------------------------------------------------

    def apply(self, left, right):
        """Controller motor units -> what the hardware actually needs.

        Trims the two sides apart, then lifts each nonzero magnitude above the
        stiction deadband. A zero command stays exactly zero: the car must never
        creep while it is stopped.
        """
        left = _clamp(left * (1.0 + self.straightness_trim))
        right = _clamp(right * (1.0 - self.straightness_trim))
        return _lift(left, self.drive_deadband), _lift(right, self.drive_deadband)

    # --- persistence ------------------------------------------------------

    def save(self, path):
        """Write to `path` as YAML, stamping the time if it is not set."""
        if not self.calibrated_at:
            self.calibrated_at = datetime.now().isoformat(timespec="seconds")
        with open(path, "w") as f:
            yaml.safe_dump(asdict(self), f, sort_keys=False)

    @classmethod
    def load(cls, path):
        """Read `path`; a missing or empty file gives pass-through defaults."""
        try:
            with open(path) as f:
                data = yaml.safe_load(f) or {}
        except FileNotFoundError:
            return cls()
        known = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in data.items() if k in known})


def _clamp(value, limit=100.0):
    return max(-limit, min(limit, value))


def _lift(value, deadband):
    """Map a magnitude of 1..100 onto deadband..100, preserving sign."""
    magnitude = abs(value)
    if magnitude < 1.0:
        return 0
    span = 100.0 - deadband
    lifted = deadband + magnitude * span / 100.0
    return int(round(lifted if value > 0 else -lifted))


# --- deriving the constants from measurements -----------------------------

def detect_deadband(samples, threshold):
    """[(motor_unit, displacement)] ascending -> the first unit that moved.

    Returns None if nothing in the ramp ever exceeded `threshold`, which means
    the ramp did not go high enough (or the car is stuck).
    """
    for unit, displacement in samples:
        if displacement >= threshold:
            return unit
    return None


def fit_gain(samples):
    """[(motor_unit, rate)] -> (gain, implied_deadband) by least squares.

    `gain` is rate per motor unit; `implied_deadband` is where the fitted line
    crosses zero rate -- the unit at which the car would just begin to move.
    """
    if len(samples) < 2:
        raise ValueError("need at least two speeds to fit a gain")

    units = [float(u) for u, _ in samples]
    rates = [float(r) for _, r in samples]
    n = len(units)
    mean_u = sum(units) / n
    mean_r = sum(rates) / n
    covariance = sum((u - mean_u) * (r - mean_r) for u, r in zip(units, rates))
    variance = sum((u - mean_u) ** 2 for u in units)
    if variance == 0.0:
        raise ValueError("all samples used the same motor unit")

    gain = covariance / variance
    if abs(gain) < 1e-9:
        raise ValueError("no measurable response: rate did not change with speed")

    intercept = mean_r - gain * mean_u
    return gain, -intercept / gain


def trim_from_drift(drift, distance, turn_sign):
    """Heading drift (rad) over a straight run of `distance` m -> a trim.

    The sign is chosen so that applying the trim opposes the curve; `turn_sign`
    carries this car's rotation convention. Bounded, because a wild reading
    (a bumped wheel, a pose glitch) must not produce a violent correction.
    """
    if distance <= 0.0:
        raise ValueError("distance must be positive")
    trim = TRIM_GAIN * turn_sign * drift / distance
    return _clamp(trim, TRIM_LIMIT)


def latency_from_step(trace, command_time):
    """[(timestamp, speed)] after a step command -> seconds until it moved.

    "Moved" is a fifth of the run's steady speed, which keeps pose noise from
    reading as motion. Returns None if the car never got going.
    """
    speeds = [s for _, s in trace]
    steady = max(speeds) if speeds else 0.0
    if steady <= 0.0:
        return None
    threshold = STEP_FRACTION * steady
    for timestamp, speed in trace:
        if speed >= threshold:
            return timestamp - command_time
    return None
