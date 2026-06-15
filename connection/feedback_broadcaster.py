from __future__ import annotations

import json
import math
import threading
import time
from dataclasses import dataclass
from typing import Any

from websockets.sync.server import Server, ServerConnection, serve

from .protocol import Feedback


@dataclass(frozen=True, slots=True)
class FeedbackBroadcastConfig:
    host: str = "127.0.0.1"
    port: int = 8766


class FeedbackBroadcaster:
    """把已解析反馈帧转发给浏览器 WebSocket 客户端。"""

    def __init__(self, cfg: FeedbackBroadcastConfig | None = None) -> None:
        self.cfg = cfg or FeedbackBroadcastConfig()
        self._server: Server | None = None
        self._server_thread: threading.Thread | None = None
        self._clients: set[ServerConnection] = set()
        self._lock = threading.Lock()
        self._condition = threading.Condition()
        self._latest_feedback: Feedback | None = None
        self._latest_seq = 0
        self._closed = False

    @property
    def url(self) -> str:
        return f"ws://{self.cfg.host}:{self.cfg.port}"

    def start(self) -> None:
        if self._server is not None:
            raise RuntimeError("反馈转发 WebSocket 服务已经启动。")
        with self._condition:
            self._closed = False
        self._server = serve(self._handle_client, self.cfg.host, self.cfg.port)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

    def stop(self) -> None:
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        server = self._server
        if server is not None:
            server.shutdown()
        if self._server_thread is not None:
            self._server_thread.join(timeout=0.5)
        with self._lock:
            clients = list(self._clients)
            self._clients.clear()
        for client in clients:
            try:
                client.close()
            except Exception:
                pass
        self._server = None
        self._server_thread = None

    def publish(self, feedback: Feedback) -> None:
        with self._condition:
            self._latest_feedback = feedback
            self._latest_seq += 1
            self._condition.notify_all()

    def _handle_client(self, websocket: ServerConnection) -> None:
        with self._lock:
            self._clients.add(websocket)
        last_seq = 0
        try:
            while True:
                with self._condition:
                    self._condition.wait_for(
                        lambda: self._closed or self._latest_seq != last_seq
                    )
                    if self._closed:
                        break
                    feedback = self._latest_feedback
                    last_seq = self._latest_seq
                if feedback is None:
                    continue
                websocket.send(json.dumps(_feedback_payload(feedback), separators=(",", ":")))
        except Exception:
            pass
        finally:
            with self._lock:
                self._clients.discard(websocket)


def _feedback_payload(feedback: Feedback) -> dict[str, Any]:
    return {
        "t": time.time(),
        "x": feedback.x,
        "y": feedback.y,
        "yaw_rad": feedback.yaw,
        "yaw_deg": math.degrees(feedback.yaw),
        "h": feedback.h,
        "q1_rad": feedback.q1,
        "q1_deg": math.degrees(feedback.q1),
        "q2_rad": feedback.q2,
        "q2_deg": math.degrees(feedback.q2),
    }
