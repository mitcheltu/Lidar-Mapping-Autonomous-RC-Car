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
