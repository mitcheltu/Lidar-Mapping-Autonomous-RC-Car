import numpy as np

from nav.drift import icp_correction, transform_points, voxel_downsample


def structured_cloud(n=800, seed=0):
    """A 3D-structured cloud (two perpendicular walls + floor) so ICP is well
    constrained in every axis."""
    rng = np.random.default_rng(seed)
    floor = np.column_stack([rng.uniform(0, 1, n), np.zeros(n), rng.uniform(0, 1, n)])
    wall_x = np.column_stack([np.zeros(n), rng.uniform(0, 1, n), rng.uniform(0, 1, n)])
    wall_z = np.column_stack([rng.uniform(0, 1, n), rng.uniform(0, 1, n), np.zeros(n)])
    return np.vstack([floor, wall_x, wall_z]).astype(np.float32)


def test_icp_recovers_a_small_translation():
    target = structured_cloud()
    offset = np.array([0.05, -0.03, 0.04])
    source = target + offset
    T = icp_correction(source, target, max_dist=0.2)
    # applying T to source aligns it to target -> T translation ~ -offset
    assert np.allclose(T[:3, 3], -offset, atol=0.02)
    aligned = transform_points(source, T)
    assert np.linalg.norm(aligned.mean(0) - target.mean(0)) < 0.02


def test_icp_identity_when_already_aligned():
    c = structured_cloud()
    T = icp_correction(c, c, max_dist=0.2)
    assert np.allclose(T, np.eye(4), atol=1e-3)


def test_icp_returns_identity_for_tiny_clouds():
    assert np.allclose(icp_correction(np.zeros((3, 3)), np.zeros((3, 3))), np.eye(4))


def test_voxel_downsample_reduces_dense_cloud():
    dense = np.random.default_rng(0).uniform(0, 0.1, (2000, 3)).astype(np.float32)
    out = voxel_downsample(dense, voxel_size=0.05)
    assert out.shape[0] < dense.shape[0]
    assert out.shape[0] <= 8   # 0.1/0.05 = 2 cells per axis -> <= 2^3 voxels


def test_transform_points_applies_rotation_and_translation():
    T = np.eye(4)
    T[:3, 3] = [1, 2, 3]
    out = transform_points(np.array([[0.0, 0.0, 0.0]]), T)
    assert np.allclose(out[0], [1, 2, 3])
