import numpy as np
import pytest

from nav.mapping import clean_cloud


def dense_plane(n_side=60, spacing=0.02, y=0.0):
    """A dense flat floor patch: n_side x n_side points."""
    xs, zs = np.meshgrid(np.arange(n_side) * spacing, np.arange(n_side) * spacing)
    pts = np.stack([xs.ravel(), np.full(xs.size, y), zs.ravel()], axis=1)
    return pts.astype(np.float32)


def test_clean_cloud_removes_isolated_outliers():
    floor = dense_plane()
    outliers = np.array([[5.0, 3.0, 5.0], [-4.0, 2.5, -4.0], [0.5, 9.0, 0.5]],
                        dtype=np.float32)
    cleaned = clean_cloud(np.vstack([floor, outliers]), voxel_size=0.04)
    assert cleaned.shape[0] > 100                    # kept the floor
    assert cleaned[:, 1].max() < 1.0                 # dropped the flying points


def test_clean_cloud_downsamples_dense_regions():
    floor = dense_plane(spacing=0.005)               # much denser than voxel size
    cleaned = clean_cloud(floor, voxel_size=0.04)
    assert cleaned.shape[0] < floor.shape[0] * 0.25


def test_clean_cloud_passes_tiny_clouds_through():
    tiny = np.random.rand(10, 3).astype(np.float32)
    out = clean_cloud(tiny)
    assert out.shape == (10, 3)


from nav.mapping import estimate_floor_height


def room_with_table(floor_y=-1.4):
    """Floor plus a smaller 'table top' plane 0.7 m above it."""
    floor = dense_plane(n_side=60, y=floor_y)
    table = dense_plane(n_side=20, y=floor_y + 0.7)
    rng = np.random.default_rng(0)
    noise = np.stack([rng.uniform(0, 1.2, 40),
                      rng.uniform(floor_y, floor_y + 1.0, 40),
                      rng.uniform(0, 1.2, 40)], axis=1).astype(np.float32)
    return np.vstack([floor, table, noise])


def test_floor_is_lowest_dominant_plane_not_the_table():
    pts = room_with_table(floor_y=-1.4)
    assert abs(estimate_floor_height(pts) - (-1.4)) < 0.05


def test_floor_ignores_sparse_low_noise():
    pts = room_with_table(floor_y=0.0)
    low_specks = np.array([[0.1, -0.9, 0.1], [0.9, -0.8, 0.9]], dtype=np.float32)
    assert abs(estimate_floor_height(np.vstack([pts, low_specks]))) < 0.05


from nav.grid import FREE, OCCUPIED, UNKNOWN
from nav.mapping import build_occupancy_grid, inflate


def box_room(floor_y=0.0, size=2.0, spacing=0.025):
    """Floor with a 0.3 m box obstacle at the center, plus an overhang to ignore."""
    n = int(size / spacing)
    floor = dense_plane(n_side=n, spacing=spacing, y=floor_y)
    bx, bz = np.meshgrid(np.arange(0.9, 1.1, spacing), np.arange(0.9, 1.1, spacing))
    box = []
    for h in (0.10, 0.20, 0.30):                    # box within robot height
        box.append(np.stack([bx.ravel(), np.full(bx.size, floor_y + h),
                             bz.ravel()], axis=1))
    # a 'shelf' 1 m up over x=0.2..0.4 -- too high to matter, must be ignored
    sx, sz = np.meshgrid(np.arange(0.2, 0.4, spacing), np.arange(0.2, 0.4, spacing))
    shelf = np.stack([sx.ravel(), np.full(sx.size, floor_y + 1.0), sz.ravel()], axis=1)
    return np.vstack([floor] + box + [shelf]).astype(np.float32)


def test_grid_classifies_free_occupied_unknown():
    pts = box_room()
    g = build_occupancy_grid(pts, floor_y=0.0, cell_size=0.05)
    assert g.cells[g.world_to_cell(1.0, 1.0)] == OCCUPIED   # the box
    assert g.cells[g.world_to_cell(0.3, 1.5)] == FREE       # open floor
    assert g.cells[g.world_to_cell(0.3, 0.3)] == FREE       # under the high shelf
    assert g.cells[g.world_to_cell(-0.4, -0.4)] == UNKNOWN  # outside scanned area


def test_inflation_blocks_a_ring_around_obstacles():
    pts = box_room()
    g = inflate(build_occupancy_grid(pts, floor_y=0.0, cell_size=0.05),
                robot_radius=0.12)
    assert g.blocked[g.world_to_cell(1.0, 1.0)]             # obstacle itself
    assert g.blocked[g.world_to_cell(1.0, 0.82)]            # within 0.12 m of box
    assert not g.blocked[g.world_to_cell(1.0, 0.5)]         # well clear of it
    assert g.passable()[g.world_to_cell(1.0, 0.5)]


def test_grid_raises_on_empty_band():
    with pytest.raises(ValueError):
        build_occupancy_grid(np.zeros((0, 3), np.float32), floor_y=0.0)


def test_clean_cloud_drops_nonfinite_points():
    floor = dense_plane()
    bad = np.array([[np.nan, 0.0, 0.0], [0.0, np.inf, 0.0]], dtype=np.float32)
    cleaned = clean_cloud(np.vstack([floor, bad]), voxel_size=0.04)
    assert np.isfinite(cleaned).all()
    assert cleaned.shape[0] > 100


def test_floor_height_raises_on_empty_cloud():
    with pytest.raises(ValueError):
        estimate_floor_height(np.zeros((0, 3), np.float32))
