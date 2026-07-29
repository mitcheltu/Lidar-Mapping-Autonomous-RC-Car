"""Coordinate-frame conversion between ARKit and ROS.

ARKit world frame is gravity-aligned with **+Y up** (floor = x-z plane). ROS/RViz
is **+Z up** (floor = x-y plane). Publishing everything in the ROS convention makes
the point cloud, voxel cubes, pose and occupancy grid share one coherent, upright
world in RViz.

The mapping is the proper (right-handed, det +1) rotation

    ros = A @ arkit,   A = [[1, 0,  0],
                            [0, 0, -1],
                            [0, 1,  0]]

so ARKit +Y (up) -> ROS +Z (up). The `nav` algorithm library still works in the
ARKit y-up convention internally; these helpers convert only at the ROS boundary.
"""

from __future__ import annotations

import numpy as np

# ros_vec(col) = A @ arkit_vec(col)
A = np.array([[1.0, 0.0, 0.0],
              [0.0, 0.0, -1.0],
              [0.0, 1.0, 0.0]], dtype=np.float64)


def points_arkit_to_ros(xyz):
    """[N,3] ARKit (y-up) points -> ROS (z-up) points."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return xyz.astype(np.float32)
    return (xyz @ A.T).astype(np.float32)


def points_ros_to_arkit(xyz):
    """[N,3] ROS (z-up) points -> ARKit (y-up) points (inverse; A is orthonormal)."""
    xyz = np.asarray(xyz, dtype=np.float32).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return xyz.astype(np.float32)
    return (xyz @ A).astype(np.float32)


def rotation_arkit_to_ros(R):
    """Re-express a 3x3 orientation from the ARKit frame in the ROS frame."""
    R = np.asarray(R, dtype=np.float64)
    return A @ R @ A.T


def rotation_ros_to_arkit(R):
    """Re-express a 3x3 orientation from the ROS frame back in the ARKit frame."""
    R = np.asarray(R, dtype=np.float64)
    return A.T @ R @ A


def matrix_to_quaternion(m):
    """3x3 rotation matrix -> quaternion (x, y, z, w), numerically stable."""
    m = np.asarray(m, dtype=np.float64)
    t = m[0, 0] + m[1, 1] + m[2, 2]
    if t > 0.0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (m[2, 1] - m[1, 2]) * s
        y = (m[0, 2] - m[2, 0]) * s
        z = (m[1, 0] - m[0, 1]) * s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        w = (m[2, 1] - m[1, 2]) / s
        x = 0.25 * s
        y = (m[0, 1] + m[1, 0]) / s
        z = (m[0, 2] + m[2, 0]) / s
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        w = (m[0, 2] - m[2, 0]) / s
        x = (m[0, 1] + m[1, 0]) / s
        y = 0.25 * s
        z = (m[1, 2] + m[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        w = (m[1, 0] - m[0, 1]) / s
        x = (m[0, 2] + m[2, 0]) / s
        y = (m[1, 2] + m[2, 1]) / s
        z = 0.25 * s
    q = np.array([x, y, z, w], dtype=np.float64)
    n = np.linalg.norm(q)
    return q / n if n > 0 else np.array([0.0, 0.0, 0.0, 1.0])


def quaternion_to_matrix(qx, qy, qz, qw):
    """Unit quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    q = np.array([qx, qy, qz, qw], dtype=np.float64)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
