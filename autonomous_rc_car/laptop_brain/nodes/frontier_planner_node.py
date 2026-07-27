#!/usr/bin/env python3
"""Choose a frontier target from an occupancy grid."""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nav.frontier import choose_goal
from nav.grid import OccupancyGrid


def run(grid: OccupancyGrid, car_cell):
    return choose_goal(grid, car_cell)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Pick the next frontier target")
    parser.add_argument("--grid", type=Path, required=True)
    parser.add_argument("--row", type=int, required=True)
    parser.add_argument("--col", type=int, required=True)
    args = parser.parse_args()

    data = np.load(args.grid)
    cells = data["cells"]
    grid = OccupancyGrid(cells=cells, origin=tuple(data["origin"]), cell_size=float(data["cell_size"][0]))
    print(run(grid, (args.row, args.col)))
