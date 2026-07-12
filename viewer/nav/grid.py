"""2D occupancy grid over the floor (x-z) plane. Rows index z, cols index x."""

from dataclasses import dataclass

import numpy as np

UNKNOWN = np.int8(-1)
FREE = np.int8(0)
OCCUPIED = np.int8(1)


@dataclass(eq=False)
class OccupancyGrid:
    cells: np.ndarray          # int8 [rows, cols] of UNKNOWN / FREE / OCCUPIED
    origin: tuple              # world (x, z) of the corner of cell [0, 0]
    cell_size: float           # meters per cell
    blocked: np.ndarray = None # bool [rows, cols]; obstacles inflated by robot radius

    @property
    def shape(self):
        return self.cells.shape

    def world_to_cell(self, x, z):
        col = int(np.floor((x - self.origin[0]) / self.cell_size))
        row = int(np.floor((z - self.origin[1]) / self.cell_size))
        return row, col

    def cell_to_world(self, row, col):
        x = self.origin[0] + (col + 0.5) * self.cell_size
        z = self.origin[1] + (row + 0.5) * self.cell_size
        return x, z

    def in_bounds(self, row, col):
        return 0 <= row < self.cells.shape[0] and 0 <= col < self.cells.shape[1]

    def passable(self):
        """Bool mask of cells the robot may occupy: FREE and outside inflation."""
        if self.blocked is None:
            raise ValueError("grid not inflated yet -- call mapping.inflate() first")
        return (self.cells == FREE) & ~self.blocked
