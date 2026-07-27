#!/usr/bin/env python3
"""Send a small synthetic point-cloud packet to the live viewer for smoke testing."""

import argparse
import socket
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from nodes.stream_protocol import encode_point_cloud_packet

HOST = "127.0.0.1"
PORT = 9002


def main():
    parser = argparse.ArgumentParser(description="Send a synthetic point-cloud packet")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=PORT)
    args = parser.parse_args()

    points = np.array([[0.0, 0.1, 0.0], [0.1, 0.2, 0.1], [0.2, 0.1, 0.2]], dtype=np.float32)
    colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    frame = encode_point_cloud_packet(points, colors)

    with socket.create_connection((args.host, args.port), timeout=2.0) as sock:
        sock.sendall(frame)
    print(f"sent {len(points)} points")


if __name__ == "__main__":
    main()
