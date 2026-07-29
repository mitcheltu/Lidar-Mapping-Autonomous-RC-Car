"""Scan-to-map ICP drift correction (Open3D).

ARKit VIO drifts slowly and can jump after tracking loss. Aligning the current
local scan to the accumulated map with ICP yields a small rigid correction that
snaps the pose back onto the map (Technical Spec sec. 3). Frame-agnostic: the
caller runs it in whatever frame its clouds live in (the ROS node uses the ROS
frame directly) and applies the returned 4x4 to poses.
"""

from __future__ import annotations

import numpy as np
import open3d as o3d


def voxel_downsample(xyz, voxel_size=0.05):
    """Keep one point per voxel (numpy, no Open3D object churn)."""
    xyz = np.asarray(xyz, dtype=np.float32)
    if xyz.shape[0] == 0:
        return xyz
    idx = np.floor(xyz / voxel_size).astype(np.int64)
    _, keep = np.unique(idx, axis=0, return_index=True)
    return xyz[np.sort(keep)]


def _pcd(xyz):
    p = o3d.geometry.PointCloud()
    p.points = o3d.utility.Vector3dVector(np.asarray(xyz, dtype=np.float64))
    return p


def icp_correction(source_xyz, target_xyz, max_dist=0.10, init=None,
                   max_iter=30):
    """Rigid 4x4 transform aligning ``source`` onto ``target`` (point-to-point
    ICP). Returns identity if either cloud is too small to register."""
    src = np.asarray(source_xyz, dtype=np.float64)
    tgt = np.asarray(target_xyz, dtype=np.float64)
    if src.shape[0] < 10 or tgt.shape[0] < 10:
        return np.eye(4)
    init = np.eye(4) if init is None else np.asarray(init, dtype=np.float64)
    reg = o3d.pipelines.registration.registration_icp(
        _pcd(src), _pcd(tgt), max_dist, init,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=max_iter),
    )
    return np.asarray(reg.transformation, dtype=np.float64)


def transform_points(xyz, T):
    """Apply a 4x4 homogeneous transform to [N,3] points."""
    xyz = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
    if xyz.shape[0] == 0:
        return xyz.astype(np.float32)
    T = np.asarray(T, dtype=np.float64)
    out = xyz @ T[:3, :3].T + T[:3, 3]
    return out.astype(np.float32)
