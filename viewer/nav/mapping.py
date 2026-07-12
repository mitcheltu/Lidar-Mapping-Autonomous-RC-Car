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


def build_occupancy_grid(xyz, floor_y, cell_size=0.05, floor_band=0.04,
                         clearance=0.06, robot_height=0.35,
                         min_obstacle_points=2, padding=0.5):
    """Collapse a cleaned 3D cloud into a 2D occupancy grid on the x-z plane.

    Height bands (all relative to floor_y):
      |y - floor| <= floor_band                -> evidence of drivable floor
      clearance < y - floor < robot_height     -> obstacle the body would hit
      above robot_height / below floor - 0.15  -> ignored (overhangs / noise)
    """
    y = xyz[:, 1]
    keep = (y > floor_y - 0.15) & (y < floor_y + robot_height)
    pts = xyz[keep]
    if pts.shape[0] == 0:
        raise ValueError("no points in the navigation height band")

    x0 = float(pts[:, 0].min()) - padding
    z0 = float(pts[:, 2].min()) - padding
    cols = int(np.ceil((float(pts[:, 0].max()) + padding - x0) / cell_size))
    rows = int(np.ceil((float(pts[:, 2].max()) + padding - z0) / cell_size))

    col_idx = np.clip(((pts[:, 0] - x0) / cell_size).astype(int), 0, cols - 1)
    row_idx = np.clip(((pts[:, 2] - z0) / cell_size).astype(int), 0, rows - 1)
    flat = row_idx * cols + col_idx

    is_floor = np.abs(pts[:, 1] - floor_y) <= floor_band
    is_obstacle = ((pts[:, 1] > floor_y + clearance) &
                   (pts[:, 1] < floor_y + robot_height))
    floor_count = np.bincount(flat[is_floor], minlength=rows * cols).reshape(rows, cols)
    obst_count = np.bincount(flat[is_obstacle], minlength=rows * cols).reshape(rows, cols)

    cells = np.full((rows, cols), UNKNOWN, dtype=np.int8)
    cells[floor_count > 0] = FREE
    cells[obst_count >= min_obstacle_points] = OCCUPIED
    return OccupancyGrid(cells=cells, origin=(x0, z0), cell_size=cell_size)


def inflate(grid, robot_radius=0.12):
    """Circular inflation: block every cell within robot_radius of an obstacle.

    Uses a Euclidean distance transform so inflation is a true disk, not the
    diamond that repeated binary dilation gives.
    """
    dist = ndimage.distance_transform_edt(grid.cells != OCCUPIED) * grid.cell_size
    grid.blocked = dist <= robot_radius
    return grid
