import numpy as np

import math

from nav.frames import (
    A,
    matrix_to_quaternion,
    points_arkit_to_ros,
    points_ros_to_arkit,
    quaternion_to_matrix,
    rotation_arkit_to_ros,
    rotation_ros_to_arkit,
)


def test_A_is_a_proper_rotation():
    assert np.allclose(A @ A.T, np.eye(3))       # orthonormal
    assert np.isclose(np.linalg.det(A), 1.0)     # right-handed (not a mirror)


def test_arkit_up_maps_to_ros_up():
    # ARKit +Y (up) -> ROS +Z (up)
    assert np.allclose(points_arkit_to_ros([[0, 1, 0]])[0], [0, 0, 1])


def test_floor_plane_stays_flat():
    # points on the ARKit floor (x-z plane, y=const) map to a ROS x-y plane (z=const)
    floor = np.array([[1.0, -1.4, 2.0], [-0.5, -1.4, 0.3]], dtype=np.float32)
    ros = points_arkit_to_ros(floor)
    assert np.allclose(ros[:, 2], -1.4)          # constant height in ROS z


def test_round_trip_is_identity():
    rng = np.random.default_rng(0)
    pts = rng.uniform(-3, 3, size=(50, 3)).astype(np.float32)
    assert np.allclose(points_ros_to_arkit(points_arkit_to_ros(pts)), pts, atol=1e-5)


def test_empty_cloud_passthrough():
    assert points_arkit_to_ros(np.zeros((0, 3), np.float32)).shape == (0, 3)


def test_rotation_identity_and_validity():
    assert np.allclose(rotation_arkit_to_ros(np.eye(3)), np.eye(3))
    R = rotation_arkit_to_ros(A)                 # arbitrary valid rotation in
    assert np.allclose(R @ R.T, np.eye(3))       # -> still orthonormal
    assert np.isclose(np.linalg.det(R), 1.0)


def test_rotation_ros_to_arkit_is_inverse():
    rng = np.random.default_rng(1)
    # a random proper rotation via QR
    q, _ = np.linalg.qr(rng.standard_normal((3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    assert np.allclose(rotation_ros_to_arkit(rotation_arkit_to_ros(q)), q)


def test_quaternion_identity_and_90deg():
    assert np.allclose(quaternion_to_matrix(0, 0, 0, 1), np.eye(3))
    # +90 deg about z maps +x -> +y
    s = math.sin(math.pi / 4)
    R = quaternion_to_matrix(0, 0, s, s)
    assert np.allclose(R @ [1, 0, 0], [0, 1, 0], atol=1e-9)


def test_matrix_quaternion_round_trip():
    rng = np.random.default_rng(3)
    q, _ = np.linalg.qr(rng.standard_normal((3, 3)))
    if np.linalg.det(q) < 0:
        q[:, 0] = -q[:, 0]
    quat = matrix_to_quaternion(q)
    R = quaternion_to_matrix(*quat)
    assert np.allclose(R, q, atol=1e-9)
