import numpy as np

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
