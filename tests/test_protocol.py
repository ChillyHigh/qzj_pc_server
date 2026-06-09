import math
import struct

import pytest

from connection.client import _consume_feedback_bytes
from connection.protocol import (
    CMD_PAYLOAD_FORMAT,
    CMD_SIZE,
    FEEDBACK_PAYLOAD_FORMAT,
    FEEDBACK_SIZE,
    SOF_BYTES,
    Feedback,
    ProtocolError,
    crc8_atm,
    pack_frame,
    parse_feedback,
)


def _feedback_frame(values: tuple[float, float, float, float, float, float]) -> bytes:
    payload = struct.pack(FEEDBACK_PAYLOAD_FORMAT, *values)
    return SOF_BYTES + payload + bytes((crc8_atm(payload),))


def test_pack_frame_appends_crc8_atm_over_payload() -> None:
    values = (
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
        6.0,
        7.0,
        8.0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
    )

    frame = pack_frame(*values, flags=0x03)

    assert len(frame) == CMD_SIZE
    assert frame[:2] == SOF_BYTES
    payload = struct.pack(CMD_PAYLOAD_FORMAT, *values, 0x03)
    assert frame[2:-1] == payload
    assert frame[-1] == crc8_atm(payload)


def test_parse_feedback_accepts_valid_crc_frame() -> None:
    frame = _feedback_frame((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

    feedback, consumed = parse_feedback(frame)

    assert feedback == Feedback(1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert consumed == FEEDBACK_SIZE


def test_parse_feedback_discards_bad_crc_frame_without_raising() -> None:
    frame = bytearray(_feedback_frame((1.0, 2.0, 3.0, 4.0, 5.0, 6.0)))
    frame[4] ^= 0x01

    feedback, consumed = parse_feedback(frame)

    assert feedback is None
    assert consumed == FEEDBACK_SIZE


def test_consume_feedback_bytes_counts_bad_crc_and_continues() -> None:
    bad = bytearray(_feedback_frame((1.0, 2.0, 3.0, 4.0, 5.0, 6.0)))
    bad[7] ^= 0x01
    good = _feedback_frame((7.0, 8.0, 9.0, 10.0, 11.0, 12.0))
    published: list[Feedback] = []
    drops = 0

    def record_drop() -> None:
        nonlocal drops
        drops += 1

    _consume_feedback_bytes(bytearray(), bytes(bad) + good, published.append, record_drop)

    assert drops == 1
    assert published == [Feedback(7.0, 8.0, 9.0, 10.0, 11.0, 12.0)]


def test_parse_feedback_waits_for_partial_frame() -> None:
    frame = _feedback_frame((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))

    feedback, consumed = parse_feedback(frame[:-1])

    assert feedback is None
    assert consumed == 0


def test_parse_feedback_rejects_non_finite_values_after_crc_passes() -> None:
    frame = _feedback_frame((1.0, 2.0, math.nan, 4.0, 5.0, 6.0))

    with pytest.raises(ProtocolError, match="反馈帧包含 NaN 或无穷大"):
        parse_feedback(frame)
