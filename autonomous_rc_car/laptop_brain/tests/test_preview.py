import numpy as np

from nav.preview import preview_plan


def plane(x0, x1, z0, z1, y, spacing=0.03):
    xs, zs = np.meshgrid(np.arange(x0, x1, spacing), np.arange(z0, z1, spacing))
    return np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()],
                    axis=1).astype(np.float32)


def wall(x0, x1, z0, z1):
    return np.vstack([plane(x0, x1, z0, z1, y=h) for h in (0.10, 0.20, 0.30)])


def partly_scanned_room():
    """2x2 m walled room; floor observed only for x < 1.2 (rest unexplored)."""
    return np.vstack([
        plane(0.0, 1.2, 0.0, 2.0, y=0.0),
        wall(-0.06, 0.0, -0.06, 2.06),
        wall(2.0, 2.06, -0.06, 2.06),
        wall(0.0, 2.0, -0.06, 0.0),
        wall(0.0, 2.0, 2.0, 2.06),
    ])


def test_preview_produces_grid_goal_and_path():
    res = preview_plan(partly_scanned_room(), (0.5, 1.0, 0.0))
    assert res.grid is not None and res.floor_y is not None
    assert res.goal_cell is not None
    assert len(res.path_world) >= 1
    gx, _ = res.grid.cell_to_world(*res.goal_cell)
    assert gx > 1.0                       # the goal points at the unexplored side


def test_preview_survives_thin_data():
    res = preview_plan(np.zeros((0, 3), np.float32), (0.0, 0.0, 0.0))
    assert res.grid is None and res.path_world == []


def test_preview_with_no_frontier_still_returns_the_grid():
    full = np.vstack([
        plane(0.0, 2.0, 0.0, 2.0, y=0.0),
        wall(-0.06, 0.0, -0.06, 2.06),
        wall(2.0, 2.06, -0.06, 2.06),
        wall(0.0, 2.0, -0.06, 0.0),
        wall(0.0, 2.0, 2.0, 2.06),
    ])
    res = preview_plan(full, (0.5, 1.0, 0.0))
    assert res.grid is not None           # map still drawn
    assert res.goal_cell is None          # nothing left to explore
