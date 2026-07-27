import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.voxel_viewer import build_overlay_geometry, prepare_point_cloud_geometry


def small_grid():
    cells = np.full((4, 4), UNKNOWN, dtype=np.int8)
    cells[1, 1] = FREE
    cells[1, 2] = OCCUPIED
    g = OccupancyGrid(cells=cells, origin=(0.0, 0.0), cell_size=0.1)
    g.blocked = np.zeros((4, 4), dtype=bool)
    g.blocked[1, 2] = True
    return g


def test_prepare_point_cloud_geometry_downsamples_large_clouds():
    xyz = np.random.rand(300, 3).astype(np.float32)
    rgb = np.random.rand(300, 3).astype(np.float32)

    pcd = prepare_point_cloud_geometry(xyz, rgb, max_points=50)

    points = np.asarray(pcd.points)
    colors = np.asarray(pcd.colors)
    assert points.shape[0] <= 50
    assert colors.shape[0] == points.shape[0]


def test_build_overlay_geometry_returns_grid_and_path_geometry():
    g = small_grid()
    gpts, gcol, ppts, plines = build_overlay_geometry(g, floor_y=0.0, path_world=[(0.0, 0.0), (0.2, 0.2)])

    assert gpts.shape[0] > 0
    assert gcol.shape == gpts.shape
    assert ppts.shape[0] == 2
    assert plines.shape[0] == 1
