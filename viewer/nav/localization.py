"""ARKit camera pose -> 2D floor pose (x, z, theta). +Y is up; floor is x-z."""

import math

import numpy as np


def pose_from_streamed(vals):
    """16 column-major floats (the 'O' message) -> 4x4 camera-to-world matrix."""
    return np.array(vals, dtype=np.float64).reshape(4, 4).T


def pose_to_2d(pose):
    """4x4 camera-to-world -> (x, z, theta). Camera looks down its local -Z,
    so world heading is the -Z column projected onto the floor plane."""
    M = np.asarray(pose, dtype=np.float64)
    fwd = -M[:3, 2]
    theta = math.atan2(fwd[2], fwd[0])
    return float(M[0, 3]), float(M[2, 3]), theta


def angle_diff(a, b):
    """Shortest signed angle a - b, wrapped to (-pi, pi]."""
    d = (a - b) % (2.0 * math.pi)
    if d > math.pi:
        d -= 2.0 * math.pi
    return d
