#!/usr/bin/env python3
"""Minimal ICP-style SLAM scaffold.

This node loads a point cloud from a .npy or .f32 file, cleans it, estimates
floor height, and writes a lightweight summary file. It is meant to be the first
step toward a real registration pipeline.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nav.mapping import clean_cloud, estimate_floor_height


def load_cloud(path: Path):
    if path.suffix == ".npy":
        data = np.load(path)
        if data.ndim == 2 and data.shape[1] >= 3:
            return data[:, :3].astype(np.float32)
        raise ValueError("expected a point cloud array with shape [N, 3]")
    if path.suffix == ".f32":
        arr = np.fromfile(path, dtype=np.float32)
        if arr.size % 6 == 0:
            arr = arr.reshape(-1, 6)
            return arr[:, :3].astype(np.float32)
        return arr.reshape(-1, 3).astype(np.float32)
    raise ValueError("unsupported input format; use .npy or .f32")


def run(path: Path, output: Path | None = None):
    xyz = load_cloud(path)
    cleaned = clean_cloud(xyz)
    floor_y = estimate_floor_height(cleaned)
    summary = {
        "input_points": int(xyz.shape[0]),
        "cleaned_points": int(cleaned.shape[0]),
        "floor_y": round(float(floor_y), 4),
    }
    if output is not None:
        output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a lightweight SLAM scaffold")
    parser.add_argument("--input", type=Path, required=True)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()
    print(json.dumps(run(args.input, args.output), indent=2))
