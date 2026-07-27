"""Binary framing helpers for point-cloud streaming between the iOS app and laptop viewer."""

from __future__ import annotations

import struct
from typing import Tuple

import numpy as np

MESSAGE_TYPE_POINT_CLOUD = 0x50


def encode_point_cloud_packet(points: np.ndarray, colors: np.ndarray | None = None) -> bytes:
    """Encode a point-cloud frame as a small binary packet."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be a (N, 3) float32 array")

    if colors is None:
        colors = np.zeros((points.shape[0], 3), dtype=np.float32)
    colors = np.asarray(colors, dtype=np.float32)
    if colors.shape != (points.shape[0], 3):
        raise ValueError("colors must match the point count and have shape (N, 3)")

    payload = bytearray()
    payload.extend(struct.pack("<I", int(points.shape[0])))
    for idx in range(points.shape[0]):
        payload.extend(struct.pack("<fff", float(points[idx, 0]), float(points[idx, 1]), float(points[idx, 2])))
        payload.extend(struct.pack("<BBB", int(np.clip(colors[idx, 0], 0.0, 1.0) * 255.0), int(np.clip(colors[idx, 1], 0.0, 1.0) * 255.0), int(np.clip(colors[idx, 2], 0.0, 1.0) * 255.0)))

    frame = bytearray()
    frame.extend(struct.pack("<B", MESSAGE_TYPE_POINT_CLOUD))
    frame.extend(struct.pack("<I", len(payload)))
    frame.extend(payload)
    return bytes(frame)


def decode_point_cloud_packet(frame: bytes) -> Tuple[np.ndarray, np.ndarray]:
    """Decode a point-cloud frame back into points and colors."""
    if len(frame) < 5:
        raise ValueError("frame is too short")

    mtype = frame[0]
    if mtype != MESSAGE_TYPE_POINT_CLOUD:
        raise ValueError("unexpected message type")

    length = struct.unpack("<I", frame[1:5])[0]
    payload = frame[5:]
    if len(payload) < length:
        raise ValueError("truncated payload")

    payload = payload[:length]
    if len(payload) < 4:
        raise ValueError("payload is too short")

    count = struct.unpack("<I", payload[:4])[0]
    body = payload[4:]
    if len(body) != count * 15:
        raise ValueError("payload size does not match point count")

    points = np.empty((count, 3), dtype=np.float32)
    colors = np.empty((count, 3), dtype=np.float32)
    offset = 0
    for idx in range(count):
        points[idx, :] = np.frombuffer(body[offset:offset + 12], dtype=np.float32)
        offset += 12
        colors[idx, :] = np.array([
            body[offset] / 255.0,
            body[offset + 1] / 255.0,
            body[offset + 2] / 255.0,
        ], dtype=np.float32)
        offset += 3

    return points, colors
