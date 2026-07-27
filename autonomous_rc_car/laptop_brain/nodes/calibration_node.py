#!/usr/bin/env python3
"""Simple PWM calibration wizard.

The node accepts a CSV file with columns: pwm,distance_m,angle_rad.
It fits a basic linear model for distance and angular speed and writes a YAML
file that can be used as the first draft of the robot's calibration config.
"""

import argparse
import csv
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def fit_linear(samples):
    if not samples:
        raise ValueError("at least one calibration sample is required")
    x = [row[0] for row in samples]
    y = [row[1] for row in samples]
    mean_x = sum(x) / len(x)
    mean_y = sum(y) / len(y)
    numerator = sum((xi - mean_x) * (yi - mean_y) for xi, yi in zip(x, y))
    denominator = sum((xi - mean_x) ** 2 for xi in x)
    if denominator == 0:
        raise ValueError("calibration samples do not vary in PWM")
    slope = numerator / denominator
    intercept = mean_y - slope * mean_x
    return slope, intercept


def load_samples(path: Path):
    with path.open("r", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    samples = []
    for row in rows:
        pwm = float(row["pwm"])
        distance = float(row["distance_m"])
        angle = float(row["angle_rad"])
        samples.append((pwm, distance, angle))
    return samples


def build_config(samples):
    dist_slope, dist_intercept = fit_linear([(pwm, distance) for pwm, distance, _ in samples])
    angle_slope, angle_intercept = fit_linear([(pwm, angle) for pwm, _, angle in samples])
    return {
        "PWM_MIN": min(p[0] for p in samples),
        "PWM_MAX": max(p[0] for p in samples),
        "LINEAR_GAIN": round(dist_slope, 6),
        "LINEAR_OFFSET": round(dist_intercept, 6),
        "ANGULAR_GAIN": round(angle_slope, 6),
        "ANGULAR_OFFSET": round(angle_intercept, 6),
    }


def write_config(path: Path, config):
    text = []
    for key in ("PWM_MIN", "PWM_MAX", "LINEAR_GAIN", "LINEAR_OFFSET", "ANGULAR_GAIN", "ANGULAR_OFFSET"):
        text.append(f"{key}: {config[key]}")
    path.write_text("\n".join(text) + "\n", encoding="utf-8")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Fit a simple PWM calibration model")
    parser.add_argument("--input", type=Path, default=None, help="CSV file with pwm,distance_m,angle_rad columns")
    parser.add_argument("--output", type=Path, default=None, help="YAML-like output file")
    args = parser.parse_args()

    if args.input is None:
        samples = [(80, 0.30, 0.00), (120, 0.48, 0.06), (160, 0.64, 0.11)]
    else:
        samples = load_samples(args.input)

    config = build_config(samples)
    if args.output is None:
        print("\n".join(f"{k}: {v}" for k, v in config.items()))
    else:
        write_config(args.output, config)
        print(f"wrote calibration config to {args.output}")
