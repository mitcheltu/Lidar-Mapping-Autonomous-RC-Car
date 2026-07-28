"""Runtime tunables for driving. TURN_SIGN is set during field calibration:
+1 means the command (left=+s, right=-s) rotates the car so theta INCREASES."""

TURN_SIGN = 1

ROBOT_RADIUS = 0.14      # meters, chassis half-diagonal + margin
DRIVE_SPEED = 45         # -100..100 motor units, indoor-safe cruise
TURN_SPEED = 40          # in-place rotation speed
SPIN_SPEED = 35          # slow 360-degree scan spin (slow = better LiDAR)
ARRIVE_DIST = 0.10       # meters to consider a waypoint reached
TURN_THRESHOLD = 0.44    # rad (~25 deg): above this, stop and turn in place
