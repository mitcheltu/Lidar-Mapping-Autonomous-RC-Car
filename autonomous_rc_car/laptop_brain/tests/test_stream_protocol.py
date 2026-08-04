import numpy as np
import pytest

from nav.stream_protocol import (
    MESSAGE_TYPE_IMAGE,
    MESSAGE_TYPE_POINT_CLOUD,
    MESSAGE_TYPE_POSE,
    decode_image_packet,
    decode_point_cloud_packet,
    decode_pose_packet,
    decode_pose_payload,
    encode_image_packet,
    encode_point_cloud_packet,
    encode_pose_packet,
    parse_frame,
)


def test_nodes_shim_reexports_library_protocol():
    # `from nodes.stream_protocol import ...` must still resolve to the same code.
    from nodes import stream_protocol as shim
    from nav import stream_protocol as lib
    assert shim.decode_point_cloud_packet is lib.decode_point_cloud_packet
    assert shim.MESSAGE_TYPE_POSE == lib.MESSAGE_TYPE_POSE


def test_encode_decode_roundtrip_preserves_points_and_colors():
    points = np.array([[0.0, 0.1, 0.0], [0.1, 0.2, 0.1]], dtype=np.float32)
    colors = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)

    frame = encode_point_cloud_packet(points, colors)
    decoded_points, decoded_colors = decode_point_cloud_packet(frame)

    assert decoded_points.shape == points.shape
    assert decoded_colors.shape == colors.shape
    assert np.allclose(decoded_points, points)
    assert np.allclose(decoded_colors, colors, atol=1.0 / 255)


def test_points_roundtrip_scales_to_thousands_of_points():
    rng = np.random.default_rng(0)
    points = rng.uniform(-3.0, 3.0, size=(5000, 3)).astype(np.float32)
    colors = rng.uniform(0.0, 1.0, size=(5000, 3)).astype(np.float32)

    xyz, rgb = decode_point_cloud_packet(encode_point_cloud_packet(points, colors))

    assert xyz.shape == (5000, 3)
    assert np.allclose(xyz, points)
    assert np.allclose(rgb, colors, atol=1.0 / 255)


def test_decode_rejects_truncated_payload():
    bad_frame = b"\x50\x04\x00\x00\x00" + b"\x00"
    with pytest.raises(ValueError):
        decode_point_cloud_packet(bad_frame)


def test_parse_frame_returns_type_and_payload():
    mtype, payload = parse_frame(b"\x4f\x02\x00\x00\x00ab")
    assert mtype == 0x4F
    assert payload == b"ab"


def test_pose_roundtrip_column_major_16_floats():
    mat = np.arange(16, dtype=np.float32).reshape(4, 4)
    vals = decode_pose_packet(encode_pose_packet(mat))
    assert len(vals) == 16
    assert np.allclose(vals, mat.ravel())


def test_pose_payload_rejects_short_buffer():
    with pytest.raises(ValueError):
        decode_pose_payload(b"\x00" * 8)


def test_image_packet_roundtrips_raw_bytes():
    jpeg = bytes(range(256)) * 3
    frame = encode_image_packet(jpeg)
    assert frame[0] == MESSAGE_TYPE_IMAGE
    assert decode_image_packet(frame) == jpeg


def test_message_type_constants_match_ascii():
    assert MESSAGE_TYPE_POINT_CLOUD == ord("P")
    assert MESSAGE_TYPE_POSE == ord("O")
    assert MESSAGE_TYPE_IMAGE == ord("I")


# --- sensor mode (laptop -> phone) ----------------------------------------

def test_mode_packet_roundtrips():
    from nav.stream_protocol import decode_mode_packet, encode_mode_packet
    assert decode_mode_packet(encode_mode_packet("SCAN")) == "SCAN"


def test_every_defined_mode_survives_the_wire():
    from nav.stream_protocol import MODES, decode_mode_packet, encode_mode_packet
    for mode in MODES:
        assert decode_mode_packet(encode_mode_packet(mode)) == mode


def test_mode_is_normalised_to_upper_case():
    from nav.stream_protocol import decode_mode_packet, encode_mode_packet
    assert decode_mode_packet(encode_mode_packet("scan")) == "SCAN"


def test_unknown_mode_is_rejected_before_it_reaches_the_phone():
    import pytest as _pytest

    from nav.stream_protocol import encode_mode_packet
    with _pytest.raises(ValueError):
        encode_mode_packet("TURBO")


def test_mode_packet_uses_its_own_message_type():
    from nav.stream_protocol import (MESSAGE_TYPE_MODE, encode_mode_packet,
                                     parse_frame)
    mtype, _ = parse_frame(encode_mode_packet("IDLE"))
    assert mtype == MESSAGE_TYPE_MODE


def test_decoding_a_non_mode_frame_is_rejected():
    import numpy as _np
    import pytest as _pytest

    from nav.stream_protocol import decode_mode_packet, encode_point_cloud_packet
    with _pytest.raises(ValueError):
        decode_mode_packet(encode_point_cloud_packet(_np.zeros((1, 3), dtype=_np.float32)))
