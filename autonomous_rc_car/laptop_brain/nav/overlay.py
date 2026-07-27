"""Turn nav state into flat colored geometry for the Open3D viewer window."""

import numpy as np

from nav.grid import OCCUPIED, UNKNOWN


def grid_overlay(grid, floor_y):
    """(points Nx3, colors Nx3) for every observed cell, drawn 1 cm above the
    floor: green = passable, amber = inflation buffer, red = obstacle."""
    rr, cc = np.nonzero(grid.cells != UNKNOWN)
    if rr.size == 0:
        return np.zeros((0, 3)), np.zeros((0, 3))
    xs = grid.origin[0] + (cc + 0.5) * grid.cell_size
    zs = grid.origin[1] + (rr + 0.5) * grid.cell_size
    pts = np.stack([xs, np.full(rr.size, floor_y + 0.01), zs], axis=1)

    colors = np.zeros((rr.size, 3))
    occ = grid.cells[rr, cc] == OCCUPIED
    blk = grid.blocked[rr, cc] & ~occ
    colors[~occ & ~blk] = (0.15, 0.55, 0.20)
    colors[blk] = (0.75, 0.55, 0.10)
    colors[occ] = (0.90, 0.15, 0.15)
    return pts, colors


def path_overlay(path_world, floor_y):
    """(points Nx3, lines Mx2) for an Open3D LineSet of the planned path,
    drawn 3 cm above the floor so it reads over the grid overlay."""
    if not path_world:
        return np.zeros((0, 3)), np.zeros((0, 2), dtype=np.int32)
    pts = np.array([[x, floor_y + 0.03, z] for x, z in path_world])
    lines = np.array([[i, i + 1] for i in range(len(pts) - 1)],
                     dtype=np.int32).reshape(-1, 2)
    return pts, lines
