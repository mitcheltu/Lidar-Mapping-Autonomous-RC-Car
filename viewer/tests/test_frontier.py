import numpy as np

from nav.grid import OccupancyGrid, FREE, OCCUPIED, UNKNOWN
from nav.frontier import bfs_distances, choose_goal, frontier_mask, nearest_passable


def open_grid(rows=20, cols=20):
    g = OccupancyGrid(cells=np.full((rows, cols), FREE, dtype=np.int8),
                      origin=(0.0, 0.0), cell_size=0.05)
    g.blocked = np.zeros((rows, cols), dtype=bool)
    return g


def test_frontier_is_free_cells_touching_unknown():
    g = open_grid()
    g.cells[:, 10:] = UNKNOWN            # right half unexplored
    m = frontier_mask(g)
    assert m[5, 9]                        # free cell bordering unknown
    assert not m[5, 5]                    # interior free cell
    assert not m[5, 12]                   # unknown cell itself


def test_blocked_cells_are_not_frontiers():
    g = open_grid()
    g.cells[:, 10:] = UNKNOWN
    g.blocked[:, 9] = True                # inflation covers the border column
    assert not frontier_mask(g).any()


def test_nearest_passable_snaps_out_of_inflation():
    g = open_grid()
    g.blocked[4:7, 4:7] = True
    assert nearest_passable(g, (5, 5)) is not None
    assert not g.blocked[nearest_passable(g, (5, 5))]


def test_bfs_goes_around_walls():
    g = open_grid()
    g.cells[0:19, 10] = OCCUPIED          # wall with a gap at the bottom
    g.blocked[0:19, 10] = True
    dist = bfs_distances(g, (0, 0))
    assert np.isfinite(dist[0, 15])       # reachable around the gap
    assert dist[0, 15] > 20               # but much farther than straight-line


def test_choose_goal_prefers_reachable_frontier():
    g = open_grid()
    g.cells[:, 15:] = UNKNOWN             # frontier on the right
    g.cells[0:20, 12] = OCCUPIED          # fully sealed off by a wall
    g.blocked[0:20, 12] = True
    assert choose_goal(g, (10, 2)) is None  # only frontier is unreachable

    g2 = open_grid()
    g2.cells[:, 15:] = UNKNOWN
    goal = choose_goal(g2, (10, 2))
    assert goal is not None
    assert goal[1] == 14                  # a frontier-column cell
    assert frontier_mask(g2)[goal]


def test_choose_goal_ignores_tiny_clusters():
    g = open_grid()
    g.cells[10, 10] = UNKNOWN             # a speck of unknown (e.g. one missing
    # floor sample) rings itself with 8 frontier cells -- the default cluster
    # threshold must be big enough to ignore it
    assert choose_goal(g, (2, 2)) is None


def test_fully_explored_returns_none():
    g = open_grid()
    assert choose_goal(g, (5, 5)) is None
