import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.ros_export import (
    ROS_FREE,
    ROS_INFLATED,
    ROS_OCCUPIED,
    ROS_UNKNOWN,
    categorize_points,
    grid_to_occupancy,
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


def test_grid_to_occupancy_metadata_matches_grid():
    exp = grid_to_occupancy(make_grid())
    assert exp.width == 4 and exp.height == 3
    assert exp.resolution == 0.05
    assert exp.origin_x == -0.5 and exp.origin_y == -1.0
    assert len(exp.data) == 12


def test_grid_to_occupancy_value_mapping():
    exp = grid_to_occupancy(make_grid())
    data = np.array(exp.data, dtype=np.int8).reshape(3, 4)
    assert data[0, 0] == ROS_FREE          # free
    assert data[0, 1] == ROS_OCCUPIED      # occupied wins even though blocked
    assert data[1, 1] == ROS_INFLATED      # free + inflated -> 99
    assert data[2, 3] == ROS_UNKNOWN       # untouched unknown -> -1


def test_grid_to_occupancy_is_row_major_z_then_x():
    # data[row*width + col] ordering (row = z index, col = x index)
    exp = grid_to_occupancy(make_grid())
    assert exp.data[0 * 4 + 1] == ROS_OCCUPIED   # cells[0,1]
    assert exp.data[1 * 4 + 1] == ROS_INFLATED   # cells[1,1]


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
