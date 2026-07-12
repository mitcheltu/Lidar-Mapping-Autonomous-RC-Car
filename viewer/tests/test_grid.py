import numpy as np
import pytest

from nav.grid import OccupancyGrid, UNKNOWN, FREE, OCCUPIED


def make_grid():
    cells = np.full((10, 20), UNKNOWN, dtype=np.int8)
    return OccupancyGrid(cells=cells, origin=(-1.0, -2.0), cell_size=0.05)


def test_world_to_cell_maps_origin_corner_to_zero():
    g = make_grid()
    assert g.world_to_cell(-1.0, -2.0) == (0, 0)


def test_world_to_cell_rows_are_z_cols_are_x():
    g = make_grid()
    # x = -1.0 + 3 cells * 0.05, z = -2.0 + 7 cells * 0.05 (plus a hair inside)
    row, col = g.world_to_cell(-1.0 + 0.16, -2.0 + 0.36)
    assert (row, col) == (7, 3)


def test_world_to_cell_before_origin_goes_negative():
    g = make_grid()
    # points before the origin map to negative indices (and are out of bounds)
    assert g.world_to_cell(-1.5, -3.0) == (-20, -10)
    assert not g.in_bounds(*g.world_to_cell(-1.5, -3.0))


def test_cell_to_world_returns_cell_center_and_round_trips():
    g = make_grid()
    x, z = g.cell_to_world(7, 3)
    assert x == pytest.approx(-1.0 + 3.5 * 0.05)
    assert z == pytest.approx(-2.0 + 7.5 * 0.05)
    assert g.world_to_cell(x, z) == (7, 3)


def test_in_bounds():
    g = make_grid()
    assert g.in_bounds(0, 0) and g.in_bounds(9, 19)
    assert not g.in_bounds(-1, 0) and not g.in_bounds(10, 0) and not g.in_bounds(0, 20)


def test_passable_requires_inflation():
    g = make_grid()
    with pytest.raises(ValueError):
        g.passable()


def test_passable_is_free_and_not_blocked():
    g = make_grid()
    g.cells[2, 2] = FREE
    g.cells[2, 3] = FREE
    g.cells[2, 4] = OCCUPIED
    g.blocked = np.zeros_like(g.cells, dtype=bool)
    g.blocked[2, 3] = True   # inflated zone
    p = g.passable()
    assert p[2, 2] and not p[2, 3] and not p[2, 4]
    assert not p[0, 0]  # unknown is never passable
