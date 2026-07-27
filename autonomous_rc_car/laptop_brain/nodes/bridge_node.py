#!/usr/bin/env python3
"""Simple bridge node for future iOS-to-ESP32 communication.

This version is intentionally lightweight: it accepts JSON command messages over
TCP, prints them for now, and can easily be extended to forward them to a real
ESP32 websocket client.
"""

import argparse
import json
import socket
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def serve(host: str = "0.0.0.0", port: int = 9001, verbose: bool = False) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, port))
    server.listen(1)
    print(f"[bridge] listening on {host}:{port}")
    try:
        while True:
            conn, addr = server.accept()
            print(f"[bridge] client connected from {addr}")
            with conn:
                while True:
                    data = conn.recv(4096)
                    if not data:
                        break
                    payload = data.decode().strip()
                    if not payload:
                        continue
                    try:
                        msg = json.loads(payload)
                    except json.JSONDecodeError:
                        msg = {"raw": payload}
                    print(json.dumps(msg, indent=2))
                    if verbose:
                        print(f"[bridge] parsed command: {msg}")
    finally:
        server.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Lightweight bridge node")
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=9001)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()
    serve(host=args.host, port=args.port, verbose=args.verbose)
