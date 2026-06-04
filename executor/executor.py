from __future__ import annotations

from client import Client
from motion_compiler import CompiledMotion, MotionCompiler
from motions import PresetMotion


class MotionExecutor:
    """执行预设动作的薄调度器。

    它只负责调用 `MotionCompiler`、下发 `CommandStream`、等待到位反馈。
    不做 IK，不做 TOPPRA，不打包协议帧。
    """

    def __init__(self, client: Client, compiler: MotionCompiler) -> None:
        """创建执行器。"""

        self.client = client
        self.compiler = compiler

    def run(self, motion: PresetMotion, *, wait_done: bool = True, timeout_s: float = 2.0) -> bool:
        """编译并执行一个预设动作。"""

        compiled = self.compiler.compile(self.client.state, motion)
        return self.run_compiled(compiled, wait_done=wait_done, timeout_s=timeout_s)

    def run_compiled(self, compiled: CompiledMotion, *, wait_done: bool = True, timeout_s: float = 2.0) -> bool:
        """执行已经编译好的动作。"""

        self.client.send_stream(compiled.stream, rate_hz=self.compiler.sample_freq)
        if not wait_done:
            return True
        return self.client.wait_done(timeout_s=timeout_s)
