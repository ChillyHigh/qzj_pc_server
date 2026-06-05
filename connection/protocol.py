from __future__ import annotations

import math
import struct
from dataclasses import dataclass

SOF = 0xAA55

FLAG_UPPER_FUNNEL_OPEN = 0x01
FLAG_LOWER_FUNNEL_OPEN = 0x02

CMD_FLOAT_COUNT = 14
FEEDBACK_FLOAT_COUNT = 6
CMD_FORMAT = "<H14fB"
CMD_SIZE = struct.calcsize(CMD_FORMAT)
FEEDBACK_FORMAT = "<H6f"
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FORMAT)
SOF_BYTES = struct.pack("<H", SOF)


class ProtocolError(ValueError):
    """协议帧非法，调用方必须停止使用本帧数据。"""


@dataclass(frozen=True, slots=True)
class Feedback:
    """下位机或 MuJoCo 桥接反馈。"""

    x: float
    y: float
    yaw: float
    h: float
    q1: float
    q2: float


def pack_frame(
    x: float,
    y: float,
    yaw: float,
    h: float,
    q1: float,
    q2: float,
    gripper_yaw: float,
    gripper_opening: float,
    dx: float,
    dy: float,
    dyaw: float,
    dh: float,
    dq1: float,
    dq2: float,
    flags: int,
) -> bytes:
    """打包统一控制帧。"""

    values = (
        x,
        y,
        yaw,
        h,
        q1,
        q2,
        gripper_yaw,
        gripper_opening,
        dx,
        dy,
        dyaw,
        dh,
        dq1,
        dq2,
    )
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("控制帧包含 NaN 或无穷大。")
    if not 0 <= flags <= 0xFF:
        raise ProtocolError(f"flags 超出 8 位范围：{flags}")
    return struct.pack(CMD_FORMAT, SOF, *values, flags & 0xFF)


def parse_feedback(data: bytes | bytearray) -> tuple[Feedback | None, int]:
    """从字节流解析一帧反馈。"""

    idx = data.find(SOF_BYTES)
    if idx < 0:
        return None, max(0, len(data) - 1)
    if idx > 0:
        return None, idx
    if len(data) < FEEDBACK_SIZE:
        return None, 0

    unpacked = struct.unpack(FEEDBACK_FORMAT, bytes(data[:FEEDBACK_SIZE]))
    sof = unpacked[0]
    if sof != SOF:
        raise ProtocolError(f"反馈帧头错误：{sof:#06x}")
    values = unpacked[1:]
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("反馈帧包含 NaN 或无穷大。")
    return Feedback(*values), FEEDBACK_SIZE
