from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Callable

import numpy as np
import serial

from command import CommandStream
from protocol import Feedback, pack_frame, parse_feedback


@dataclass(frozen=True, slots=True)
class MachineState:
    """执行层维护的最近目标状态。

    字段单位与协议完全一致。执行层合并底盘和 arm 轨迹时，以该状态补齐未运动轴。
    """

    x: float = 0.3
    y: float = 0.0
    yaw: float = 0.0
    h: float = 0.3
    q1: float = -0.5235987755982988
    q2: float = -2.6179938779914944
    gripper_yaw: float = 0.0
    gripper_opening: float = 0.0
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
    """通信策略接口。

    `send_frame` 必须完整发送一个二进制控制帧；反馈通过后台线程解析并发布。
    """

    def __init__(self) -> None:
        self._feedback: Feedback | None = None
        self._callback: Callable[[Feedback], None] | None = None

    @property
    def feedback(self) -> Feedback | None:
        """返回最近一帧反馈；没有收到反馈时为 None。"""

        return self._feedback

    def set_feedback_callback(self, callback: Callable[[Feedback], None]) -> None:
        """设置反馈回调；每收到完整反馈帧调用一次。"""

        self._callback = callback

    def clear_feedback(self) -> None:
        """清空最近反馈，用于新动作开始前避免读取旧到位状态。"""

        self._feedback = None

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
        while self._running:
            if self._ser is None:
                time.sleep(0.01)
                continue
            data = self._ser.read(256)
            if not data:
                continue
            buf.extend(data)
            if len(buf) > 1024:
                del buf[: len(buf) - 1024]
            while buf:
                feedback, consumed = parse_feedback(buf)
                if consumed > 0:
                    del buf[:consumed]
                if feedback is None:
                    break
                self._publish(feedback)


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
        while self._running:
            if self._ws is None:
                time.sleep(0.01)
                continue
            try:
                data = self._ws.recv(timeout=self.cfg.recv_timeout)
            except TimeoutError:
                continue
            except Exception:
                break
            if not isinstance(data, (bytes, bytearray)):
                continue
            buf.extend(data)
            if len(buf) > 1024:
                del buf[: len(buf) - 1024]
            while buf:
                feedback, consumed = parse_feedback(buf)
                if consumed > 0:
                    del buf[:consumed]
                if feedback is None:
                    break
                self._publish(feedback)


class Client:
    """统一下发客户端。

    底盘和 arm 不拆串口。调用方传入已经合成好的 `CommandStream`，
    Client 只按统一协议打包发送，不关心 TOPPRA、IK 或动作来源。
    """

    def __init__(self, transport: Transport, *, state: MachineState | None = None) -> None:
        """创建客户端。

        Args:
            transport: 串口或 websocket 通信策略。
            state: 执行层已知的最近目标状态；为空时使用仿真初始状态。
        """

        self.transport = transport
        self.state = state or MachineState()

    @property
    def feedback(self) -> Feedback | None:
        """最近反馈帧。"""

        return self.transport.feedback

    def connect(self) -> bool:
        """连接通信后端。"""

        return self.transport.connect()

    def close(self) -> None:
        """关闭通信后端。"""

        self.transport.close()

    def set_feedback_callback(self, callback: Callable[[Feedback], None]) -> None:
        """设置反馈回调。"""

        self.transport.set_feedback_callback(callback)

    def send_stream(self, stream: CommandStream, *, rate_hz: float | None = None) -> None:
        """按统一协议发送整车命令流。

        `stream.q` 和 `stream.dq` 已经包含完整整车状态，通信层不再合并底盘/arm。
        """

        period_s = self._period(stream, rate_hz)
        next_time = time.perf_counter()
        self.transport.clear_feedback()

        for idx in range(stream.point_count):
            frame = self._frame_at(stream, idx)
            self.transport.send_frame(frame)
            if period_s <= 0.0:
                continue
            next_time += period_s
            sleep_s = next_time - time.perf_counter()
            if sleep_s > 0.0:
                time.sleep(sleep_s)

        self._commit_state(stream)

    def wait_done(self, timeout_s: float = 2.0, poll_s: float = 0.02) -> bool:
        """等待反馈到位。

        MuJoCo 桥接和下位机均应在 `flags` 中置位 `FLAG_DONE`。
        当前没有反馈时会等待到超时，返回 False。
        """

        from protocol import FLAG_DONE

        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            feedback = self.feedback
            if feedback is not None and feedback.flags & FLAG_DONE:
                return True
            time.sleep(poll_s)
        return False

    def _frame_at(
        self,
        stream: CommandStream,
        idx: int,
    ) -> bytes:
        q = stream.q[idx]
        dq = stream.dq[idx]

        return pack_frame(
            float(q[0]),
            float(q[1]),
            float(q[2]),
            float(q[3]),
            float(q[4]),
            float(q[5]),
            float(q[6]),
            float(q[7]),
            float(dq[0]),
            float(dq[1]),
            float(dq[2]),
            float(dq[3]),
            float(dq[4]),
            float(dq[5]),
            int(stream.flags[idx]),
        )

    def _commit_state(self, stream: CommandStream) -> None:
        q = stream.q[-1]
        self.state = MachineState(
            float(q[0]),
            float(q[1]),
            float(q[2]),
            float(q[3]),
            float(q[4]),
            float(q[5]),
            float(q[6]),
            float(q[7]),
            int(stream.flags[-1]),
        )

    def _period(self, stream: CommandStream, rate_hz: float | None) -> float:
        if rate_hz is not None:
            if rate_hz <= 0.0:
                return 0.0
            return 1.0 / rate_hz
        if stream.point_count < 2:
            return 0.0
        diffs = np.diff(stream.t)
        return float(np.median(diffs))
