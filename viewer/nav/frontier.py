"""Frontier-based exploration (Yamauchi 1997): go to the free/unknown boundary."""

from collections import deque

import numpy as np
from scipy import ndimage

from nav.grid import FREE, UNKNOWN

_EIGHT = np.ones((3, 3), dtype=bool)


def frontier_mask(grid):
    """Passable FREE cells that touch at least one UNKNOWN cell (8-connected)."""
    unknown = grid.cells == UNKNOWN
    near_unknown = ndimage.binary_dilation(unknown, structure=_EIGHT)
    return grid.passable() & near_unknown


def nearest_passable(grid, cell, max_radius=12):
    """The passable cell closest to `cell` (itself if already passable).

    The car's own cell can sit inside the inflation ring right after a scan
    (walls near the start), so planning snaps to the nearest legal cell.
    After a hit, the search keeps expanding into later Chebyshev rings until
    a ring's minimum possible distance provably cannot beat the best found.
    """
    p = grid.passable()
    r0, c0 = cell
    best, best_d2 = None, None
    for radius in range(max_radius + 1):
        if best is not None and radius * radius > best_d2:
            break                     # no later ring can contain a closer cell
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                if max(abs(dr), abs(dc)) != radius:
                    continue
                r, c = r0 + dr, c0 + dc
                if grid.in_bounds(r, c) and p[r, c]:
                    d2 = dr * dr + dc * dc
                    if best is None or d2 < best_d2:
                        best, best_d2 = (r, c), d2
    return best


def bfs_distances(grid, start):
    """Steps from `start` to every passable cell (8-connected); inf if unreachable."""
    passable = grid.passable()
    dist = np.full(grid.shape, np.inf)
    if not grid.in_bounds(*start) or not passable[start]:
        return dist
    dist[start] = 0.0
    q = deque([start])
    while q:
        r, c = q.popleft()
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                if dr == 0 and dc == 0:
                    continue
                n = (r + dr, c + dc)
                if grid.in_bounds(*n) and passable[n] and not np.isfinite(dist[n]):
                    dist[n] = dist[r, c] + 1
                    q.append(n)
    return dist


def choose_goal(grid, car_cell, min_cluster_size=12):
    """Nearest-frontier goal: the reachable frontier cell of the closest
    big-enough cluster. Returns a (row, col) cell, or None when exploration
    is complete (no reachable frontier remains).

    min_cluster_size=12 because even a single unexplored cell rings itself
    with 8 frontier cells; a real doorway/room-edge frontier at 5 cm cells
    is dozens of cells wide."""
    fmask = frontier_mask(grid)
    labels, n_clusters = ndimage.label(fmask, structure=_EIGHT.astype(int))
    if n_clusters == 0:
        return None
    start = nearest_passable(grid, car_cell)
    if start is None:
        return None
    dist = bfs_distances(grid, start)

    best_d, best_cell = np.inf, None
    for lab in range(1, n_clusters + 1):
        cells = np.argwhere(labels == lab)
        if len(cells) < min_cluster_size:
            continue
        ds = dist[cells[:, 0], cells[:, 1]]
        i = int(np.argmin(ds))
        if np.isfinite(ds[i]) and ds[i] < best_d:
            best_d = ds[i]
            best_cell = (int(cells[i, 0]), int(cells[i, 1]))
    return best_cell
