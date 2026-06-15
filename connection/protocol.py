from __future__ import annotations

import math
import struct
from dataclasses import dataclass

SOF = 0xAA55

FLAG_UPPER_FUNNEL_OPEN = 0x01
FLAG_LOWER_FUNNEL_OPEN = 0x02

CMD_FLOAT_COUNT = 14
FEEDBACK_FLOAT_COUNT = 6
CRC_SIZE = 1
CMD_PAYLOAD_FORMAT = "<14fB"
CMD_FORMAT = "<H14fBB"
CMD_SIZE = struct.calcsize(CMD_FORMAT)
FEEDBACK_PAYLOAD_FORMAT = "<6f"
FEEDBACK_FORMAT = "<H6fB"
FEEDBACK_SIZE = struct.calcsize(FEEDBACK_FORMAT)
SOF_BYTES = struct.pack("<H", SOF)


class ProtocolError(ValueError):
    """协议帧非法，调用方必须停止使用本帧数据。"""


def crc8_atm(data: bytes | bytearray) -> int:
    """计算 CRC-8/ATM，poly=0x07, init=0x00, xorout=0x00。"""

    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


@dataclass(frozen=True, slots=True)
class Feedback:
    """下位机或 MuJoCo 桥接反馈。

    x/y/yaw 使用四主动轮对角线交点。
    Python 侧内部 yaw/q1/q2 使用 rad；下位机反馈帧 yaw/q1/q2 使用 deg。
    """

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
    """打包统一控制帧。

    Python 侧内部统一使用 rad/rad/s；下位机协议使用 deg/deg/s 接收角度量。
    因此只在这里把角度目标和角速度前馈转成下位机单位。
    x/y 使用四主动轮对角线交点。
    """

    values = (
        x,
        y,
        math.degrees(yaw),
        h,
        math.degrees(q1),
        math.degrees(q2),
        math.degrees(gripper_yaw),
        math.degrees(gripper_opening),
        dx,
        dy,
        math.degrees(dyaw),
        dh,
        math.degrees(dq1),
        math.degrees(dq2),
    )
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("控制帧包含 NaN 或无穷大。")
    if not 0 <= flags <= 0xFF:
        raise ProtocolError(f"flags 超出 8 位范围：{flags}")
    payload = struct.pack(CMD_PAYLOAD_FORMAT, *values, flags & 0xFF)
    return SOF_BYTES + payload + bytes((crc8_atm(payload),))


def parse_feedback(data: bytes | bytearray) -> tuple[Feedback | None, int]:
    """从字节流解析一帧反馈。"""

    idx = data.find(SOF_BYTES)
    if idx < 0:
        return None, max(0, len(data) - 1)
    if idx > 0:
        return None, idx
    if len(data) < FEEDBACK_SIZE:
        return None, 0

    frame = bytes(data[:FEEDBACK_SIZE])
    sof = struct.unpack("<H", frame[:2])[0]
    if sof != SOF:
        raise ProtocolError(f"反馈帧头错误：{sof:#06x}")
    payload = frame[2:-CRC_SIZE]
    expected_crc = frame[-CRC_SIZE]
    actual_crc = crc8_atm(payload)
    if actual_crc != expected_crc:
        return None, FEEDBACK_SIZE

    x, y, yaw_deg, h, q1_deg, q2_deg = struct.unpack(FEEDBACK_PAYLOAD_FORMAT, payload)
    values = (x, y, yaw_deg, h, q1_deg, q2_deg)
    if not all(math.isfinite(value) for value in values):
        raise ProtocolError("反馈帧包含 NaN 或无穷大。")
    return (
        Feedback(x, y, math.radians(yaw_deg), h, math.radians(q1_deg), math.radians(q2_deg)),
        FEEDBACK_SIZE,
    )
