"""Binary framing for the iPhone <-> laptop stream (points, pose, image).

One wire protocol shared by the viewer and the ROS2 bridge node. Lives in the
`nav` library so both the standalone viewer and the (pip-installed) ROS2 package
import the same definition. Each message is framed as: 1 byte type + uint32
little-endian payload length + payload. Message types match the iOS app:

  'P' 0x50 points : uint32 count, then per point  float32 x,y,z + uint8 r,g,b  (15 B/pt)
  'O' 0x4F pose   : 16 float32, the camera 4x4 transform (column-major)
  'I' 0x49 image  : raw JPEG bytes of a downscaled camera frame

Phone -> laptop for those three. The reverse direction carries commands:

  'D' 0x44 drive  : ASCII "L<left>R<right>" (legacy BLE relay path)
  'M' 0x4D mode   : ASCII "IDLE" | "SCAN" | "DRIVE" -- when the phone should
                    spend power on depth. Unknown types are ignored by both
                    ends, so an un-updated app keeps working unchanged.
"""

from __future__ import annotations

import struct
from typing import Tuple

import numpy as np

__all__ = [
    "MESSAGE_TYPE_POINT_CLOUD", "MESSAGE_TYPE_POSE", "MESSAGE_TYPE_IMAGE",
    "POINT_STRIDE", "frame", "parse_frame",
    "encode_points_payload", "decode_points_payload",
    "encode_point_cloud_packet", "decode_point_cloud_packet",
    "encode_pose_payload", "decode_pose_payload",
    "encode_pose_packet", "decode_pose_packet",
    "encode_image_packet", "decode_image_packet",
    "MESSAGE_TYPE_MODE", "MODES", "MODE_IDLE", "MODE_SCAN", "MODE_DRIVE",
    "encode_mode_packet", "decode_mode_packet",
]

MESSAGE_TYPE_POINT_CLOUD = 0x50  # 'P'
MESSAGE_TYPE_POSE = 0x4F         # 'O'
MESSAGE_TYPE_IMAGE = 0x49        # 'I'
MESSAGE_TYPE_MODE = 0x4D         # 'M'

POINT_STRIDE = 15                # 12 bytes xyz float32 + 3 bytes rgb uint8

# What the phone should be spending power on. IDLE is the default and the point
# of the exercise: pose is cheap, unprojecting depth is not.
MODE_IDLE = "IDLE"    # pose only
MODE_SCAN = "SCAN"    # depth, during a commanded stationary 360-degree spin
MODE_DRIVE = "DRIVE"  # depth, while the car is moving
MODES = (MODE_IDLE, MODE_SCAN, MODE_DRIVE)


# --- framing ---------------------------------------------------------------

def frame(mtype: int, payload: bytes) -> bytes:
    """Wrap a payload with the 1-byte type + uint32 LE length header."""
    return struct.pack("<BI", mtype, len(payload)) + payload


def parse_frame(buf: bytes) -> Tuple[int, bytes]:
    """Split a framed message into (message_type, payload). Raises ValueError
    if the buffer is shorter than the header or its declared payload length."""
    if len(buf) < 5:
        raise ValueError("frame is too short")
    mtype = buf[0]
    length = struct.unpack("<I", buf[1:5])[0]
    payload = buf[5:]
    if len(payload) < length:
        raise ValueError("truncated payload")
    return mtype, payload[:length]


# --- point cloud -----------------------------------------------------------

def encode_points_payload(points: np.ndarray, colors: np.ndarray | None = None) -> bytes:
    """Pack an (N,3) cloud into  uint32 count + N*(xyz float32, rgb uint8)."""
    points = np.asarray(points, dtype=np.float32)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must be a (N, 3) float32 array")
    n = points.shape[0]
    if colors is None:
        colors = np.zeros((n, 3), dtype=np.float32)
    colors = np.asarray(colors, dtype=np.float32)
    if colors.shape != (n, 3):
        raise ValueError("colors must match the point count and have shape (N, 3)")

    body = np.empty((n, POINT_STRIDE), dtype=np.uint8)
    body[:, :12] = np.ascontiguousarray(points.astype("<f4")).view(np.uint8).reshape(n, 12)
    body[:, 12:15] = (np.clip(colors, 0.0, 1.0) * 255.0).astype(np.uint8)
    return struct.pack("<I", n) + body.tobytes()


def decode_points_payload(payload: bytes) -> Tuple[np.ndarray, np.ndarray]:
    """Vectorized inverse of encode_points_payload. Returns (xyz float32 [N,3],
    rgb float32 [N,3] in 0..1)."""
    if len(payload) < 4:
        raise ValueError("payload is too short")
    count = struct.unpack("<I", payload[:4])[0]
    body = np.frombuffer(payload[4:], dtype=np.uint8)
    if body.size < count * POINT_STRIDE:
        raise ValueError("payload size does not match point count")
    body = body[: count * POINT_STRIDE].reshape(count, POINT_STRIDE)
    xyz = np.ascontiguousarray(body[:, :12]).view(np.float32).reshape(count, 3).copy()
    rgb = body[:, 12:15].astype(np.float32) / 255.0
    return xyz, rgb


def encode_point_cloud_packet(points: np.ndarray, colors: np.ndarray | None = None) -> bytes:
    return frame(MESSAGE_TYPE_POINT_CLOUD, encode_points_payload(points, colors))


def decode_point_cloud_packet(buf: bytes) -> Tuple[np.ndarray, np.ndarray]:
    mtype, payload = parse_frame(buf)
    if mtype != MESSAGE_TYPE_POINT_CLOUD:
        raise ValueError("unexpected message type")
    return decode_points_payload(payload)


# --- pose ------------------------------------------------------------------

def encode_pose_payload(matrix) -> bytes:
    """Pack a 4x4 (or flat 16-element) transform as 16 float32 LE."""
    vals = np.asarray(matrix, dtype="<f4").ravel()
    if vals.size != 16:
        raise ValueError("pose must have 16 elements")
    return vals.tobytes()


def decode_pose_payload(payload: bytes) -> Tuple[float, ...]:
    """Return the 16 pose floats (column-major 4x4, as the app sends them)."""
    if len(payload) < 64:
        raise ValueError("pose payload must be 64 bytes")
    return struct.unpack("<16f", payload[:64])


def encode_pose_packet(matrix) -> bytes:
    return frame(MESSAGE_TYPE_POSE, encode_pose_payload(matrix))


def decode_pose_packet(buf: bytes) -> Tuple[float, ...]:
    mtype, payload = parse_frame(buf)
    if mtype != MESSAGE_TYPE_POSE:
        raise ValueError("unexpected message type")
    return decode_pose_payload(payload)


# --- image -----------------------------------------------------------------

def encode_image_packet(jpeg: bytes) -> bytes:
    return frame(MESSAGE_TYPE_IMAGE, bytes(jpeg))


def decode_image_packet(buf: bytes) -> bytes:
    mtype, payload = parse_frame(buf)
    if mtype != MESSAGE_TYPE_IMAGE:
        raise ValueError("unexpected message type")
    return payload


# --- sensor mode (laptop -> phone) -----------------------------------------

def encode_mode_packet(mode: str) -> bytes:
    """Frame a sensor mode command. Rejects anything not in MODES rather than
    letting a typo reach the phone, where it would be silently ignored."""
    normalised = str(mode).strip().upper()
    if normalised not in MODES:
        raise ValueError(f"unknown sensor mode {mode!r}; expected one of {MODES}")
    return frame(MESSAGE_TYPE_MODE, normalised.encode("ascii"))


def decode_mode_packet(buf: bytes) -> str:
    mtype, payload = parse_frame(buf)
    if mtype != MESSAGE_TYPE_MODE:
        raise ValueError("unexpected message type")
    return payload.decode("ascii").strip().upper()
