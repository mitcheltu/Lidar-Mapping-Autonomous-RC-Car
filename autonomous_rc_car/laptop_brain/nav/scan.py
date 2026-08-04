"""Tracking a spin-in-place scan.

The car turns on the spot while the phone streams depth, and something has to
decide when it has come all the way round. Heading from ``pose_to_2d`` wraps at
+-pi, so comparing raw angles is useless past half a turn; what works is summing
the shortest signed step between consecutive readings.

Summing *signed* steps also means a car rocking back and forth on the spot never
accumulates a phantom rotation -- the steps cancel, which is the honest answer.

ROS-free and unit-testable; ``scan_node`` is a thin wrapper around this.
"""

from __future__ import annotations

import math

from nav.localization import angle_diff


class RotationTracker:
    """Accumulated turn, fed one heading at a time.

    ``turned`` is signed (positive and negative are the two directions); the
    target is compared against its magnitude, so a scan completes whichever way
    the car happens to rotate.
    """

    def __init__(self, start: float, target: float = 2.0 * math.pi):
        if target <= 0.0:
            raise ValueError("target must be positive")
        self._previous = float(start)
        self.target = float(target)
        self.turned = 0.0

    def update(self, heading: float) -> float:
        """Feed the latest heading; returns the new accumulated rotation."""
        heading = float(heading)
        self.turned += angle_diff(heading, self._previous)
        self._previous = heading
        return self.turned

    @property
    def done(self) -> bool:
        return abs(self.turned) >= self.target

    @property
    def progress(self) -> float:
        """0..1 fraction of the target turn completed."""
        return min(1.0, abs(self.turned) / self.target)

    @property
    def degrees(self) -> float:
        return abs(math.degrees(self.turned))

    @property
    def target_degrees(self) -> float:
        return math.degrees(self.target)
