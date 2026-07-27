"""Helpers that turn the current point-cloud + grid state into Open3D geometry."""

from __future__ import annotations

import numpy as np
import open3d as o3d

from nav.grid import OCCUPIED, UNKNOWN
from nav.overlay import grid_overlay, path_overlay


def prepare_point_cloud_geometry(xyz: np.ndarray, rgb: np.ndarray, max_points: int = 250_000, voxel_size: float = 0.02):
    """Create a display-ready point cloud and downsample when needed."""
    xyz = np.asarray(xyz, dtype=np.float32)
    rgb = np.asarray(rgb, dtype=np.float32)
    if xyz.shape[0] == 0:
        return o3d.geometry.PointCloud()

    if xyz.shape[0] > max_points:
        stride = max(1, xyz.shape[0] // max_points)
        xyz = xyz[::stride]
        rgb = rgb[::stride]

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd.colors = o3d.utility.Vector3dVector(rgb.astype(np.float64))
    if xyz.shape[0] > 50:
        pcd = pcd.voxel_down_sample(voxel_size=voxel_size)
    return pcd


def build_overlay_geometry(grid, floor_y: float, path_world=None):
    """Return the geometry payload needed by the viewer: grid points/colors and path lines."""
    gpts, gcol = grid_overlay(grid, floor_y)
    if path_world is None:
        path_world = []
    ppts, plines = path_overlay(path_world, floor_y)
    return gpts, gcol, ppts, plines
