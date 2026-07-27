#!/usr/bin/env python3
"""Build an occupancy grid from a point cloud.

This node uses the existing navigation helpers to estimate the floor height,
build a 2D occupancy grid, and save the result to disk as a numpy archive.
"""

import argparse
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nav.grid import FREE, OCCUPIED, UNKNOWN
from nav.mapping import build_occupancy_grid, clean_cloud, estimate_floor_height, inflate


def run(path: Path, output: Path | None = None):
    xyz = np.load(path) if path.suffix == ".npy" else np.fromfile(path, dtype=np.float32).reshape(-1, 3)
    cleaned = clean_cloud(xyz)
    floor_y = estimate_floor_height(cleaned)
    grid = build_occupancy_grid(cleaned, floor_y)
    inflate(grid)
    if output is not None:
        np.savez_compressed(output, cells=grid.cells, origin=np.array(grid.origin, dtype=np.float32), cell_size=np.array([grid.cell_size], dtype=np.float32))
    return {
        "shape": list(grid.shape),
        "free": int((grid.cells == FREE).sum()),
        "occupied": int((grid.cells == OCCUPIED).sum()),
        "unknown": int((grid.cells == UNKNOWN).sum()),
    }


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Build a 2D occupancy grid")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(run(args.input, args.output))
