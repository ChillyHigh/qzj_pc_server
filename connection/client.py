from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

import serial

from .protocol import FEEDBACK_SIZE, SOF_BYTES, Feedback, pack_frame, parse_feedback


@dataclass(frozen=True, slots=True)
class MachineState:
    """执行层维护的最近目标状态。

    x/y/yaw 使用四主动轮对角线交点。
    """

    x: float = 0.3
    y: float = 0.0
    yaw: float = 0.0
    h: float = 0.3
    q1: float = 3.6651914291880923
    q2: float = 3.6651914291880923
    gripper_yaw: float = 0.0
    gripper_opening: float = 0.0
    dx: float = 0.0
    dy: float = 0.0
    dyaw: float = 0.0
    dh: float = 0.0
    dq1: float = 0.0
    dq2: float = 0.0
    flags: int = 0


@dataclass(frozen=True, slots=True)
class SerialConfig:
    """串口配置。"""

    port: str
    baud: int = 115200
    timeout: float = 0.05


@dataclass(frozen=True, slots=True)
class WebSocketConfig:
    """WebSocket 配置。"""

    url: str
    open_timeout: float = 5.0
    recv_timeout: float = 0.05
    ping_interval: float | None = None


class Transport(ABC):
    """通信策略接口。"""

    def __init__(self) -> None:
        self._feedback: Feedback | None = None
        self._error: Exception | None = None
        self._callback: Callable[[Feedback], None] | None = None
        self._feedback_crc_drop_count = 0

    @property
    def feedback(self) -> Feedback | None:
        """返回最近一帧反馈；没有收到反馈时为 None。"""

        return self._feedback

    @property
    def error(self) -> Exception | None:
        """返回后台接收错误；没有错误时为 None。"""

        return self._error

    @property
    def feedback_crc_drop_count(self) -> int:
        """返回 CRC 错误导致丢弃的反馈帧数量。"""

        return self._feedback_crc_drop_count

    def set_feedback_callback(self, callback: Callable[[Feedback], None]) -> None:
        """设置反馈回调；每收到完整反馈帧调用一次。"""

        self._callback = callback

    def clear_feedback(self) -> None:
        """清空最近反馈。"""

        self._feedback = None

    def _fail(self, exc: Exception) -> None:
        self._error = exc

    def _record_feedback_crc_drop(self) -> None:
        self._feedback_crc_drop_count += 1

    def _publish(self, feedback: Feedback) -> None:
        self._feedback = feedback
        if self._callback is not None:
            self._callback(feedback)

    @abstractmethod
    def connect(self) -> bool:
        """建立通信连接，成功返回 True。"""

    @abstractmethod
    def close(self) -> None:
        """关闭通信连接和后台接收线程。"""

    @abstractmethod
    def send_frame(self, frame: bytes) -> None:
        """发送单帧二进制控制数据。"""


class SerialTransport(Transport):
    """串口通信策略。"""

    def __init__(self, cfg: SerialConfig) -> None:
        super().__init__()
        self.cfg = cfg
        self._ser: serial.Serial | None = None
        self._rx_thread: threading.Thread | None = None
        self._running = False

    def connect(self) -> bool:
        try:
            self._ser = serial.Serial(self.cfg.port, self.cfg.baud, timeout=self.cfg.timeout)
        except Exception as exc:
            print(f"连接串口失败：端口={self.cfg.port} 波特率={self.cfg.baud} 原因={exc}")
            return False
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        return True

    def close(self) -> None:
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.5)
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def send_frame(self, frame: bytes) -> None:
        if self._ser is None:
            raise RuntimeError("串口尚未连接，不能发送控制帧。")
        self._ser.write(frame)

    def _rx_loop(self) -> None:
        buf = bytearray()
        try:
            while self._running:
                if self._ser is None:
                    raise RuntimeError("串口接收线程发现串口对象为空。")
                data = self._ser.read(256)
                if not data:
                    continue
                _consume_feedback_bytes(buf, data, self._publish, self._record_feedback_crc_drop)
        except Exception as exc:
            self._fail(exc)
            self._running = False


class WebSocketTransport(Transport):
    """WebSocket 通信策略，用于连接 MuJoCo 桥接或同格式下位机代理。"""

    def __init__(self, cfg: WebSocketConfig) -> None:
        super().__init__()
        self.cfg = cfg
        self._ws = None
        self._rx_thread: threading.Thread | None = None
        self._running = False

    def connect(self) -> bool:
        try:
            from websockets.sync.client import connect

            self._ws = connect(
                self.cfg.url,
                open_timeout=self.cfg.open_timeout,
                ping_interval=self.cfg.ping_interval,
                max_size=None,
            )
        except Exception as exc:
            print(f"连接 websocket 失败：地址={self.cfg.url} 原因={exc}")
            return False
        self._running = True
        self._rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_thread.start()
        return True

    def close(self) -> None:
        self._running = False
        if self._rx_thread is not None:
            self._rx_thread.join(timeout=0.5)
        if self._ws is not None:
            self._ws.close()
            self._ws = None

    def send_frame(self, frame: bytes) -> None:
        if self._ws is None:
            raise RuntimeError("websocket 尚未连接，不能发送控制帧。")
        self._ws.send(frame)

    def _rx_loop(self) -> None:
        buf = bytearray()
        try:
            while self._running:
                if self._ws is None:
                    raise RuntimeError("websocket 接收线程发现连接对象为空。")
                try:
                    data = self._ws.recv(timeout=self.cfg.recv_timeout)
                except TimeoutError:
                    continue
                if not isinstance(data, (bytes, bytearray)):
                    raise RuntimeError(f"websocket 收到非二进制反馈：{type(data)!r}")
                _consume_feedback_bytes(buf, data, self._publish, self._record_feedback_crc_drop)
        except Exception as exc:
            self._fail(exc)
            self._running = False


class Client:
    """统一下发客户端；只发送当前单帧，不理解 DAG 或轨迹来源。"""

    def __init__(self, transport: Transport, *, state: MachineState | None = None) -> None:
        self.transport = transport
        self.state = state or MachineState()

    @property
    def feedback(self) -> Feedback | None:
        """最近反馈帧。"""

        return self.transport.feedback

    @property
    def error(self) -> Exception | None:
        """后台接收错误。"""

        return self.transport.error

    @property
    def feedback_crc_drop_count(self) -> int:
        """CRC 错误导致丢弃的反馈帧数量。"""

        return self.transport.feedback_crc_drop_count

    def connect(self) -> bool:
        """连接通信后端。"""

        return self.transport.connect()

    def close(self) -> None:
        """关闭通信后端。"""

        self.transport.close()

    def set_feedback_callback(self, callback: Callable[[Feedback], None]) -> None:
        """设置反馈回调。"""

        self.transport.set_feedback_callback(callback)

    def clear_feedback(self) -> None:
        """清空最近反馈。"""

        self.transport.clear_feedback()

    def send_command(self, state: MachineState) -> None:
        """发送一个完整控制周期的目标状态。"""

        if self.transport.error is not None:
            raise RuntimeError(f"通信接收失败：{self.transport.error}") from self.transport.error
        frame = pack_frame(
            state.x,
            state.y,
            state.yaw,
            state.h,
            state.q1,
            state.q2,
            state.gripper_yaw,
            state.gripper_opening,
            state.dx,
            state.dy,
            state.dyaw,
            state.dh,
            state.dq1,
            state.dq2,
            state.flags,
        )
        self.transport.send_frame(frame)
        self.state = state


def _consume_feedback_bytes(
    buf: bytearray,
    data: bytes | bytearray,
    publish: Callable[[Feedback], None],
    record_crc_drop: Callable[[], None] | None = None,
) -> None:
    buf.extend(data)
    if len(buf) > 1024:
        del buf[: len(buf) - 1024]
    while buf:
        is_crc_drop = len(buf) >= FEEDBACK_SIZE and buf.startswith(SOF_BYTES)
        feedback, consumed = parse_feedback(buf)
        if consumed > 0:
            del buf[:consumed]
        if feedback is None:
            if consumed > 0 and is_crc_drop:
                if record_crc_drop is not None:
                    record_crc_drop()
                continue
            if consumed > 0:
                continue
            break
        publish(feedback)
