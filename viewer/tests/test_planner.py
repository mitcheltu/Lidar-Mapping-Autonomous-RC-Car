import numpy as np

from nav.planner import astar, line_clear, simplify_path


def open_map(rows=20, cols=20):
    return np.ones((rows, cols), dtype=bool)


def test_astar_straight_line_in_open_space():
    path = astar(open_map(), (0, 0), (0, 10))
    assert path is not None
    assert path[0] == (0, 0) and path[-1] == (0, 10)
    assert len(path) == 11


def test_astar_routes_around_a_wall():
    p = open_map()
    p[0:15, 10] = False                       # wall with gap at rows 15..19
    path = astar(p, (5, 5), (5, 15))
    assert path is not None
    assert all(p[cell] for cell in path)      # never enters a blocked cell
    assert max(r for r, _ in path) >= 15      # went around via the gap


def test_astar_no_corner_cutting():
    p = open_map(3, 3)
    p[0, 1] = p[1, 0] = False                 # diagonal squeeze at (0,0)->(1,1)
    path = astar(p, (0, 0), (2, 2))
    assert path is None                       # the only exit is a blocked squeeze


def test_astar_unreachable_returns_none():
    p = open_map()
    p[:, 10] = False                          # full wall
    assert astar(p, (5, 5), (5, 15)) is None


def test_line_clear_sees_blockers():
    p = open_map()
    assert line_clear(p, (0, 0), (10, 10))
    p[5, 5] = False
    assert not line_clear(p, (0, 0), (10, 10))


def test_simplify_collapses_collinear_and_keeps_corners():
    p = open_map()
    p[0:15, 10] = False
    path = astar(p, (5, 5), (5, 15))
    simple = simplify_path(path, p)
    assert simple[0] == path[0] and simple[-1] == path[-1]
    assert len(simple) <= 5                   # a few corners, not 30 cells
    for a, b in zip(simple, simple[1:]):
        assert line_clear(p, a, b)            # every leg is collision-free
