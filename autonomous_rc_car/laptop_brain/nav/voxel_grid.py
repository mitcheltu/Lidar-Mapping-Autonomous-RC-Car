"""Incremental log-odds voxel grid with ray carving (OctoMap-style).

Maintains a persistent ``{voxel index -> log-odds}`` map. Each ``update`` casts a
ray from the sensor origin to every point: voxels the ray passes *through* accrue
free evidence (log-odds down), the endpoint accrues occupied evidence (up). A
voxel is occupied when its log-odds exceeds ``occ_threshold``. This carves away
stale/moved obstacles the sensor now sees through, and costs O(P*L) per frame
(P = points that frame, L = voxels per ray) instead of re-voxelizing the whole
accumulated cloud every cycle.

ROS-free and unit-testable. Coordinates are the ARKit y-up world frame (same as
``nav.mapping``); the ROS node converts at its boundary.
"""

from __future__ import annotations

import numpy as np


def voxel_traversal(origin, end, size, max_steps=4096):
    """Amanatides & Woo 3D DDA. Yields integer voxel keys from the origin voxel
    to the end voxel inclusive, in order along the segment origin->end."""
    o = np.asarray(origin, dtype=np.float64)
    e = np.asarray(end, dtype=np.float64)
    d = e - o
    cur = np.floor(o / size).astype(np.int64)
    endv = np.floor(e / size).astype(np.int64)

    step = np.zeros(3, dtype=np.int64)
    tmax = np.full(3, np.inf)
    tdelta = np.full(3, np.inf)
    for i in range(3):
        if d[i] > 0:
            step[i] = 1
            tmax[i] = ((cur[i] + 1) * size - o[i]) / d[i]
            tdelta[i] = size / d[i]
        elif d[i] < 0:
            step[i] = -1
            tmax[i] = (cur[i] * size - o[i]) / d[i]
            tdelta[i] = size / -d[i]

    keys = [tuple(int(v) for v in cur)]
    end_t = tuple(int(v) for v in endv)
    for _ in range(max_steps):
        if tuple(int(v) for v in cur) == end_t:
            break
        axis = int(np.argmin(tmax))
        if tmax[axis] > 1.0:
            break
        cur[axis] += step[axis]
        tmax[axis] += tdelta[axis]
        keys.append(tuple(int(v) for v in cur))
    return keys


class VoxelGrid:
    def __init__(self, voxel_size=0.03, l_occ=0.85, l_free=0.4,
                 l_min=-2.0, l_max=3.5, occ_threshold=0.0, max_range=6.0):
        self.voxel_size = float(voxel_size)
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.occ_threshold = float(occ_threshold)
        self.max_range = float(max_range)
        self._log = {}   # (ix, iy, iz) -> log-odds

    def _voxel_of(self, p):
        return (int(np.floor(p[0] / self.voxel_size)),
                int(np.floor(p[1] / self.voxel_size)),
                int(np.floor(p[2] / self.voxel_size)))

    def _bump(self, key, delta):
        v = self._log.get(key, 0.0) + delta
        self._log[key] = min(self.l_max, max(self.l_min, v))

    def integrate_ray(self, origin, point):
        endv = self._voxel_of(point)
        for key in voxel_traversal(origin, point, self.voxel_size):
            if key == endv:
                continue
            self._bump(key, -self.l_free)   # traversed -> free evidence
        self._bump(endv, self.l_occ)        # endpoint -> occupied evidence

    def update(self, points, origin, max_rays=2000):
        """Integrate a batch of hits from ``origin`` (sensor). Range-gated, and
        the number of carved rays is capped at ``max_rays`` for bounded cost."""
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if points.shape[0] == 0:
            return
        origin = np.asarray(origin, dtype=np.float64).ravel()[:3]
        dist = np.linalg.norm(points - origin, axis=1)
        points = points[(dist > 1e-6) & (dist <= self.max_range)]
        if points.shape[0] > max_rays:
            idx = np.linspace(0, points.shape[0] - 1, max_rays).astype(int)
            points = points[idx]
        for p in points:
            self.integrate_ray(origin, p)

    def occupied_centers(self):
        """Centers [N,3] float32 of voxels whose log-odds exceed the threshold."""
        keys = [k for k, v in self._log.items() if v > self.occ_threshold]
        if not keys:
            return np.zeros((0, 3), dtype=np.float32)
        return ((np.array(keys, dtype=np.float32) + 0.5) * self.voxel_size).astype(np.float32)

    def __len__(self):
        return len(self._log)
