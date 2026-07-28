"""True 3D voxelization for visualization/perception.

Discretize a point cloud into a grid of cubes of edge ``voxel_size`` (index
``V = floor(world / voxel_size)``) and mark a voxel *occupied* when enough LiDAR
points fall inside it -- the standard "enough hits on a cube => obstacle" rule
(Technical Spec sec. 4). Voxels are classified by height band, mirroring the
occupancy-grid pipeline:

    |y - floor| <= floor_band                -> 'ground' voxel
    floor+clearance < y < floor+robot_height -> 'obstacle' voxel

Returns the voxel *centers* (float32 [N,3]); a cube of edge ``voxel_size`` is
drawn at each. ROS-free so it stays unit-testable.
"""

from __future__ import annotations

import numpy as np


def occupied_voxel_centers(pts, voxel_size, min_points):
    """Centers of voxels containing at least ``min_points`` of ``pts``."""
    pts = np.asarray(pts, dtype=np.float32)
    if pts.shape[0] == 0:
        return np.zeros((0, 3), dtype=np.float32)
    idx = np.floor(pts / voxel_size).astype(np.int64)
    uniq, counts = np.unique(idx, axis=0, return_counts=True)
    keep = uniq[counts >= min_points]
    return ((keep.astype(np.float32) + 0.5) * voxel_size).astype(np.float32)


def voxelize(xyz, floor_y, voxel_size=0.03, min_points_ground=1,
             min_points_obstacle=2, floor_band=0.04, clearance=0.06,
             robot_height=0.35):
    """Split a cloud into occupied ground/obstacle voxel centers.

    ``min_points_obstacle`` > 1 rejects lone stray hits so a cube only counts as
    an obstacle when enough LiDAR lands in it.
    """
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.size == 0:
        empty = np.zeros((0, 3), dtype=np.float32)
        return {"ground": empty, "obstacle": empty.copy()}
    y = xyz[:, 1]
    ground_pts = xyz[np.abs(y - floor_y) <= floor_band]
    obstacle_pts = xyz[(y > floor_y + clearance) & (y < floor_y + robot_height)]
    return {
        "ground": occupied_voxel_centers(ground_pts, voxel_size, min_points_ground),
        "obstacle": occupied_voxel_centers(obstacle_pts, voxel_size, min_points_obstacle),
    }
