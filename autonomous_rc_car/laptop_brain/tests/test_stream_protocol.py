import numpy as np

from nodes.stream_protocol import decode_point_cloud_packet, encode_point_cloud_packet


def test_encode_decode_roundtrip_preserves_points_and_colors():
    points = np.array([[0.0, 0.1, 0.0], [0.1, 0.2, 0.1]], dtype=np.float32)
    colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)

    frame = encode_point_cloud_packet(points, colors)
    decoded_points, decoded_colors = decode_point_cloud_packet(frame)

    assert decoded_points.shape == points.shape
    assert decoded_colors.shape == colors.shape
    assert np.allclose(decoded_points, points)
    assert np.allclose(decoded_colors, colors)


def test_decode_rejects_truncated_payload():
    bad_frame = b"\x50\x04\x00\x00\x00" + b"\x00"

    try:
        decode_point_cloud_packet(bad_frame)
    except ValueError:
        pass
    else:
        raise AssertionError("expected truncated payload to raise ValueError")
