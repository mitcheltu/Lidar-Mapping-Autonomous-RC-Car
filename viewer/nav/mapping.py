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


def estimate_floor_height(xyz, bin_size=0.02, min_fraction=0.20):
    """Height (y) of the floor: the LOWEST strongly-populated horizontal slab.

    Histogram the y values; the floor is the lowest bin whose count is at
    least `min_fraction` of the largest bin, so sparse below-floor noise is
    skipped but a big table can't win just by having more points.
    """
    ys = xyz[:, 1]
    lo, hi = np.percentile(ys, [1, 99])
    edges = np.arange(lo, hi + bin_size, bin_size)
    if edges.size < 2:
        return float(np.median(ys))
    hist, edges = np.histogram(ys, bins=edges)
    threshold = hist.max() * min_fraction
    idx = int(np.argmax(hist >= threshold))   # first (lowest) qualifying bin
    return float(edges[idx] + bin_size / 2)
