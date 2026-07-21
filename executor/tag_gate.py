from __future__ import annotations

import math
import socket
from typing import Protocol

import config
from connection.client import MachineState

GATE_CLOSED = b"0"
GATE_OPEN = b"1"


class TagGateError(RuntimeError):
    """AprilTag 下发门控通信失败。"""


class TagGate(Protocol):
    """执行器使用的 AprilTag 门控接口。"""

    def publish(self, state: MachineState, now: float) -> None:
        """根据当前底盘速度发布门控状态。"""


def is_slow(state: MachineState) -> bool:
    """平移速度和角速度均严格低于配置门限时返回 True。"""

    speeds = (state.dx, state.dy, state.dyaw)
    if not all(math.isfinite(speed) for speed in speeds):
        raise TagGateError("底盘速度包含 NaN 或无穷大。")
    return (
        math.hypot(state.dx, state.dy) < config.TAG_GATE_LINEAR_LIMIT
        and abs(state.dyaw) < config.TAG_GATE_ANGULAR_LIMIT
    )


class TagGateSender:
    """通过 Unix Domain Datagram 发布 AprilTag 下发门控状态。"""

    def __init__(self) -> None:
        if not config.TAG_GATE_UDS_PATH:
            raise ValueError("TAG_GATE_UDS_PATH 不能为空。")
        if config.TAG_GATE_LINEAR_LIMIT <= 0.0:
            raise ValueError("TAG_GATE_LINEAR_LIMIT 必须大于 0。")
        if config.TAG_GATE_ANGULAR_LIMIT <= 0.0:
            raise ValueError("TAG_GATE_ANGULAR_LIMIT 必须大于 0。")
        if config.TAG_GATE_HEARTBEAT_S <= 0.0:
            raise ValueError("TAG_GATE_HEARTBEAT_S 必须大于 0。")
        self._last_value: bytes | None = None
        self._last_send_time: float | None = None

    def publish(self, state: MachineState, now: float) -> None:
        """状态变化时立即发送，状态不变时按配置周期发送心跳。"""

        value = GATE_OPEN if is_slow(state) else GATE_CLOSED
        if (
            value == self._last_value
            and self._last_send_time is not None
            and now - self._last_send_time < config.TAG_GATE_HEARTBEAT_S
        ):
            return

        try:
            with socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM) as link:
                link.setblocking(False)
                sent = link.sendto(value, config.TAG_GATE_UDS_PATH)
        except OSError as exc:
            raise TagGateError(
                f"AprilTag 门控 UDS 发送失败：{config.TAG_GATE_UDS_PATH}：{exc}"
            ) from exc
        if sent != len(value):
            raise TagGateError(f"AprilTag 门控 UDS 发送不完整：{sent}/{len(value)}")

        self._last_value = value
        self._last_send_time = now
