"""Runtime tunables for driving.

Two kinds of value live here. The plain constants below are design choices --
how fast to cruise indoors, how close counts as arrived. The rest come from
`calibration.yaml`, measured on the real car by `calibration_node` and loaded at
import, so re-calibrating takes effect on the next run with no rebuild.

Point `RC_CAR_CALIBRATION` at a different file to override the default location.
"""

import os
from pathlib import Path

from nav.calibration import Calibration

ROBOT_RADIUS = 0.14      # meters, chassis half-diagonal + margin
DRIVE_SPEED = 45         # -100..100 motor units, indoor-safe cruise
TURN_SPEED = 40          # in-place rotation speed
SPIN_SPEED = 35          # slow 360-degree scan spin (slow = better LiDAR)
ARRIVE_DIST = 0.10       # meters to consider a waypoint reached
TURN_THRESHOLD = 0.44    # rad (~25 deg): above this, stop and turn in place

# nav/config.py -> laptop_brain -> autonomous_rc_car -> config/calibration.yaml
DEFAULT_CALIBRATION_PATH = (
    Path(__file__).resolve().parents[2] / "config" / "calibration.yaml"
)

CALIBRATION = Calibration()
TURN_SIGN = CALIBRATION.turn_sign


def calibration_path():
    """Where the calibration file is read from, honouring the env override."""
    return Path(os.environ.get("RC_CAR_CALIBRATION", DEFAULT_CALIBRATION_PATH))


def reload_calibration():
    """Re-read the calibration file into the module globals.

    Called at import, and again by `calibration_node` once it has written a fresh
    result, so a running graph picks it up without a restart.
    """
    global CALIBRATION, TURN_SIGN
    CALIBRATION = Calibration.load(calibration_path())
    TURN_SIGN = CALIBRATION.turn_sign
    return CALIBRATION


reload_calibration()
