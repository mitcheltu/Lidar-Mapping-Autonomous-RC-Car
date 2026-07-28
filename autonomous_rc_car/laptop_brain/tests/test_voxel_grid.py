import numpy as np

from nav.voxel_grid import VoxelGrid, voxel_traversal


def test_traversal_visits_cells_in_order_along_x():
    keys = voxel_traversal([0.05, 0.05, 0.05], [0.35, 0.05, 0.05], 0.1)
    assert keys[0] == (0, 0, 0)
    assert keys[-1] == (3, 0, 0)
    assert (1, 0, 0) in keys and (2, 0, 0) in keys


def test_hit_endpoint_becomes_occupied():
    g = VoxelGrid(voxel_size=0.1, occ_threshold=0.0)
    g.integrate_ray([0, 0, 0], [0.55, 0.0, 0.0])
    endv = g._voxel_of([0.55, 0.0, 0.0])
    assert g._log[endv] > 0.0
    assert g.occupied_centers().shape[0] >= 1


def test_ray_carves_a_stale_occupied_voxel():
    g = VoxelGrid(voxel_size=0.1, l_occ=0.85, l_free=0.4, occ_threshold=0.0)
    near = [0.35, 0.0, 0.0]
    v = g._voxel_of(near)
    for _ in range(3):                       # build up an obstacle at v
        g.integrate_ray([0, 0, 0], near)
    assert g._log[v] > 0.0                    # occupied

    far = [0.75, 0.0, 0.0]                    # obstacle gone: see through v
    for _ in range(10):
        g.integrate_ray([0, 0, 0], far)
    assert g._log[v] < 0.0                     # carved to free
    occ = {tuple(np.floor(c / 0.1 + 1e-6).astype(int)) for c in g.occupied_centers()}
    assert v not in occ


def test_update_range_gate_drops_far_points():
    g = VoxelGrid(voxel_size=0.1, max_range=1.0)
    g.update(np.array([[5.0, 0.0, 0.0]]), origin=[0, 0, 0])
    assert len(g) == 0


def test_update_ray_cap_bounds_work():
    g = VoxelGrid(voxel_size=0.1, max_range=100.0)
    pts = np.random.default_rng(0).uniform(-1, 1, size=(5000, 3))
    g.update(pts, origin=[0, 0, 0], max_rays=200)
    assert len(g) > 0    # something integrated, and it returned promptly
