"""Point cloud -> navigable occupancy grid pipeline (the 'costmap' builder)."""

import numpy as np
import open3d as o3d
from scipy import ndimage

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN


def clean_cloud(xyz, voxel_size=0.04, nb_neighbors=20, std_ratio=2.0,
                 radius=0.10, min_neighbors=4):
    """Voxel-downsample then drop statistical + radius outliers.

    Returns an [N, 3] float32 array. Clouds under 50 points pass through
    untouched (not enough neighbors for the statistics to mean anything).
    """
    if xyz.shape[0] < 50:
        return xyz.astype(np.float32)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz.astype(np.float64))
    pcd = pcd.voxel_down_sample(voxel_size)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors,
                                             std_ratio=std_ratio)
    pcd, _ = pcd.remove_radius_outlier(nb_points=min_neighbors, radius=radius)
    return np.asarray(pcd.points, dtype=np.float32)
