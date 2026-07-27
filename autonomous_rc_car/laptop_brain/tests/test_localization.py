import math

import numpy as np
import pytest

from nav.localization import angle_diff, pose_from_streamed, pose_to_2d


def pose_matrix(x, y, z, yaw):
    """Camera-to-world matrix for a camera at (x,y,z) whose -Z (look direction)
    points at world heading `yaw` in the x-z plane (yaw = atan2 convention)."""
    fwd = np.array([math.cos(yaw), 0.0, math.sin(yaw)])
    up = np.array([0.0, 1.0, 0.0])
    right = np.cross(fwd, up)          # camera +X
    M = np.eye(4)
    M[:3, 0] = right
    M[:3, 1] = up
    M[:3, 2] = -fwd                    # camera looks down its local -Z
    M[:3, 3] = [x, y, z]
    return M


def test_pose_to_2d_extracts_position_and_heading():
    M = pose_matrix(1.5, 0.3, -2.0, yaw=math.pi / 4)
    x, z, theta = pose_to_2d(M)
    assert (x, z) == pytest.approx((1.5, -2.0))
    assert theta == pytest.approx(math.pi / 4)


def test_pose_from_streamed_matches_viewer_convention():
    # pc_viewer treats the 16 floats as column-major: reshape(4,4).T
    M = pose_matrix(0.5, 0.0, 0.25, yaw=0.0)
    vals = tuple(M.T.ravel())          # column-major flatten
    x, z, theta = pose_to_2d(pose_from_streamed(vals))
    assert (x, z) == pytest.approx((0.5, 0.25))
    assert theta == pytest.approx(0.0)


def test_angle_diff_wraps_to_pi():
    assert angle_diff(math.pi - 0.1, -math.pi + 0.1) == pytest.approx(-0.2)
    assert angle_diff(0.1, -0.1) == pytest.approx(0.2)
