import numpy as np

from nav.voxel import occupied_voxel_centers, voxelize


def test_multiple_points_in_one_cube_collapse_to_one_voxel():
    # three points inside the same 0.03 m cube -> a single voxel center
    pts = np.array([[0.001, 0.001, 0.001],
                    [0.02, 0.02, 0.02],
                    [0.029, 0.005, 0.011]], dtype=np.float32)
    centers = occupied_voxel_centers(pts, voxel_size=0.03, min_points=1)
    assert centers.shape == (1, 3)
    assert np.allclose(centers[0], [0.015, 0.015, 0.015], atol=1e-6)


def test_min_points_threshold_rejects_sparse_cubes():
    # cube A has 2 points, cube B has 1 -> only A survives min_points=2
    pts = np.array([[0.01, 0.0, 0.01], [0.02, 0.0, 0.02],   # cube (0,0,0)
                    [0.10, 0.0, 0.10]], dtype=np.float32)     # cube (3,0,3)
    centers = occupied_voxel_centers(pts, voxel_size=0.03, min_points=2)
    assert centers.shape == (1, 3)
    assert np.allclose(centers[0], [0.015, 0.015, 0.015], atol=1e-6)


def test_voxelize_separates_ground_and_obstacle_by_height():
    floor_y = -1.4
    pts = np.array([
        [0.00, -1.40, 0.00], [0.01, -1.40, 0.01],   # ground cube (>=1 pt)
        [0.20, -1.20, 0.20], [0.205, -1.20, 0.205],  # obstacle cube (2 pts)
        [0.50, -0.90, 0.50],                          # above robot height -> ignored
    ], dtype=np.float32)
    v = voxelize(pts, floor_y, voxel_size=0.03,
                 min_points_ground=1, min_points_obstacle=2)
    assert v["ground"].shape[0] == 1
    assert v["obstacle"].shape[0] == 1
    # ground voxel sits near the floor, obstacle voxel well above it
    assert v["ground"][0][1] < floor_y + 0.05
    assert v["obstacle"][0][1] > floor_y + 0.1


def test_lone_obstacle_hit_is_dropped():
    floor_y = 0.0
    pts = np.array([[0.2, 0.2, 0.2]], dtype=np.float32)  # single obstacle-band point
    v = voxelize(pts, floor_y, min_points_obstacle=2)
    assert v["obstacle"].shape[0] == 0


def test_voxelize_handles_empty_cloud():
    v = voxelize(np.zeros((0, 3), np.float32), floor_y=0.0)
    assert v["ground"].shape == (0, 3) and v["obstacle"].shape == (0, 3)
