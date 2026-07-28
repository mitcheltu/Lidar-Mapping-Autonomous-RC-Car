import numpy as np
import pytest

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.ros_export import (
    ROS_FREE,
    ROS_INFLATED,
    ROS_OCCUPIED,
    ROS_UNKNOWN,
    categorize_points,
    grid_to_occupancy,
    occupancy_to_grid,
)


def make_grid():
    cells = np.full((3, 4), UNKNOWN, dtype=np.int8)
    cells[0, 0] = FREE
    cells[0, 1] = OCCUPIED
    cells[1, 1] = FREE
    g = OccupancyGrid(cells=cells, origin=(-0.5, -1.0), cell_size=0.05)
    g.blocked = np.zeros((3, 4), dtype=bool)
    g.blocked[1, 1] = True   # free but inside inflation
    g.blocked[0, 1] = True   # obstacle cell also flagged blocked -> stays OCCUPIED
    return g


def test_grid_to_occupancy_metadata_is_zup_ros():
    exp = grid_to_occupancy(make_grid())
    assert exp.width == 4 and exp.height == 3
    assert exp.resolution == 0.05
    assert len(exp.data) == 12
    assert exp.origin_x == pytest.approx(-0.5)
    # ROS y of the data[0] corner = -(world z origin + rows*cell)
    assert exp.origin_y == pytest.approx(-(-1.0 + 3 * 0.05))


def test_grid_to_occupancy_value_mapping():
    exp = grid_to_occupancy(make_grid())
    # un-flip the published rows to recover the nav-grid layout
    out = np.flipud(np.array(exp.data, dtype=np.int8).reshape(3, 4))
    assert out[0, 0] == ROS_FREE           # free
    assert out[0, 1] == ROS_OCCUPIED       # occupied wins even though blocked
    assert out[1, 1] == ROS_INFLATED       # free + inflated -> 99
    assert out[2, 3] == ROS_UNKNOWN        # untouched unknown -> -1


def test_grid_to_occupancy_flips_rows_for_zup():
    # world row r (z index) lands at published row (height-1-r) because +ROS y = -world z
    exp = grid_to_occupancy(make_grid())
    data = np.array(exp.data, dtype=np.int8).reshape(3, 4)
    assert data[2, 1] == ROS_OCCUPIED      # cells[0,1] -> top world row -> last data row
    assert data[1, 1] == ROS_INFLATED      # cells[1,1] -> middle stays middle


def test_categorize_points_splits_ground_and_obstacle():
    floor_y = -1.4
    pts = np.array([
        [0.0, -1.40, 0.0],   # ground (on floor)
        [0.1, -1.39, 0.1],   # ground (within band)
        [0.2, -1.20, 0.2],   # obstacle (0.2 m above floor)
        [0.3, -1.10, 0.3],   # obstacle
        [0.4, -0.90, 0.4],   # above robot_height (0.35) -> ignored
        [0.5, -1.80, 0.5],   # below floor -> ignored
    ], dtype=np.float32)
    layers = categorize_points(pts, floor_y)
    assert layers["ground"].shape[0] == 2
    assert layers["obstacle"].shape[0] == 2
    # ignored points are in neither layer
    assert layers["ground"].shape[0] + layers["obstacle"].shape[0] == 4


def test_categorize_points_handles_empty_cloud():
    layers = categorize_points(np.zeros((0, 3), np.float32), floor_y=0.0)
    assert layers["ground"].shape == (0, 3)
    assert layers["obstacle"].shape == (0, 3)


def test_occupancy_roundtrip_recovers_cells_blocked_and_origin():
    g = make_grid()
    exp = grid_to_occupancy(g)
    back = occupancy_to_grid(exp.width, exp.height, exp.resolution,
                             exp.origin_x, exp.origin_y, exp.data)
    assert np.array_equal(back.cells, g.cells)
    assert np.array_equal(back.blocked, g.blocked)
    assert back.cell_size == g.cell_size
    assert back.origin[0] == pytest.approx(g.origin[0])
    assert back.origin[1] == pytest.approx(g.origin[1])


def test_occupancy_roundtrip_preserves_passable_for_planning():
    g = make_grid()
    exp = grid_to_occupancy(g)
    back = occupancy_to_grid(exp.width, exp.height, exp.resolution,
                             exp.origin_x, exp.origin_y, exp.data)
    assert np.array_equal(back.passable(), g.passable())
