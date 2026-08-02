"""Waypoint follower: turn-then-drive local controller for differential drive.

Given the car's 2D pose (x, z, theta) and a list of (x, z) waypoints, emit a
(left, right) motor command in -100..100. Turns in place until roughly facing the
next waypoint, then drives with proportional heading correction. TURN_SIGN
(nav.config) accounts for wiring and is calibrated in the field.
"""

import math
from dataclasses import dataclass, field

from nav import config
from nav.localization import angle_diff


@dataclass
class WaypointFollower:
    waypoints: list                    # [(x, z), ...] world coordinates
    arrive_dist: float = config.ARRIVE_DIST
    turn_threshold: float = config.TURN_THRESHOLD
    drive_speed: int = config.DRIVE_SPEED
    turn_speed: int = config.TURN_SPEED
    calibration: object = None         # nav.calibration.Calibration
    _index: int = field(default=0, repr=False)

    def __post_init__(self):
        if self.calibration is None:
            self.calibration = config.CALIBRATION

    @property
    def stop_distance(self):
        """Arrival radius widened by however far the car coasts before it reacts.

        An uncalibrated car has zero gain and zero latency, so this is just
        `arrive_dist` until someone has actually measured the hardware.
        """
        lead = (self.calibration.linear_gain * self.drive_speed
                * self.calibration.command_latency)
        return self.arrive_dist + lead

    @property
    def done(self):
        return self._index >= len(self.waypoints)

    @property
    def current_waypoint(self):
        return None if self.done else self.waypoints[self._index]

    def update(self, x, z, theta):
        """Current pose -> (left, right) motor command. Call at >= 5 Hz."""
        stop_dist = self.stop_distance
        while not self.done:
            wx, wz = self.waypoints[self._index]
            if math.hypot(wx - x, wz - z) < stop_dist:
                self._index += 1
            else:
                break
        if self.done:
            return 0, 0

        wx, wz = self.waypoints[self._index]
        bearing = math.atan2(wz - z, wx - x)
        err = angle_diff(bearing, theta)

        if abs(err) > self.turn_threshold:
            s = self.turn_speed if err > 0 else -self.turn_speed
            return config.TURN_SIGN * s, -config.TURN_SIGN * s

        correction = config.TURN_SIGN * int(max(-15, min(15, err * 40)))
        return self.drive_speed - correction, self.drive_speed + correction
