import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.overlay import grid_overlay, path_overlay


def small_grid():
    cells = np.full((4, 4), UNKNOWN, dtype=np.int8)
    cells[1, 1] = FREE
    cells[1, 2] = OCCUPIED
    g = OccupancyGrid(cells=cells, origin=(0.0, 0.0), cell_size=0.1)
    g.blocked = np.zeros((4, 4), dtype=bool)
    g.blocked[1, 2] = True
    return g


def test_grid_overlay_draws_only_observed_cells_above_floor():
    pts, colors = grid_overlay(small_grid(), floor_y=-1.0)
    assert pts.shape == (2, 3) and colors.shape == (2, 3)
    assert np.allclose(pts[:, 1], -0.99)             # 1 cm above the floor
    assert not np.array_equal(colors[0], colors[1])  # free vs occupied differ


def test_grid_overlay_empty_grid():
    g = small_grid()
    g.cells[:] = UNKNOWN
    pts, colors = grid_overlay(g, floor_y=0.0)
    assert pts.shape == (0, 3) and colors.shape == (0, 3)


def test_path_overlay_builds_line_segments():
    pts, lines = path_overlay([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)], floor_y=0.0)
    assert pts.shape == (3, 3)
    assert lines.tolist() == [[0, 1], [1, 2]]


def test_path_overlay_empty():
    pts, lines = path_overlay([], floor_y=0.0)
    assert pts.shape == (0, 3) and lines.shape == (0, 2)


def test_path_overlay_single_point_keeps_line_shape():
    pts, lines = path_overlay([(1.0, 2.0)], floor_y=0.0)
    assert pts.shape == (1, 3)
    assert lines.shape == (0, 2)
