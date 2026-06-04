from __future__ import annotations

import math

from client import Client, SerialConfig, SerialTransport, WebSocketConfig, WebSocketTransport
from executor import MotionExecutor
from motion_compiler import MotionCompiler
from motions import ArmTo, DirectMove


def main() -> None:
    """执行层手动入口。

    默认连接 MuJoCo websocket；切到串口时只改 `mode` 和串口参数。
    """

    mode = "ws"
    ws_url = "ws://127.0.0.1:8765"
    port = "/dev/tty.usbserial-1110"
    baud = 230400

    if mode == "ws":
        client = Client(WebSocketTransport(WebSocketConfig(url=ws_url)))
    else:
        client = Client(SerialTransport(SerialConfig(port=port, baud=baud)))

    if not client.connect():
        raise SystemExit("无法连接通信后端。")

    try:
        compiler = MotionCompiler(sample_freq=50.0)
        executor = MotionExecutor(client, compiler)

        _run(
            compiler,
            executor,
            "底盘移动",
            DirectMove((0.45, 0.12, math.radians(8.0)), speed_scale=0.6),
        )

        _run(
            compiler,
            executor,
            "五连杆末端移动",
            ArmTo(
                endpoint_xy=(0.275, 0.045),
                h=client.state.h,
                gripper_yaw=math.radians(12.0),
                gripper_opening=math.radians(30.0),
                speed_scale=0.6,
            ),
        )

        _run(
            compiler,
            executor,
            "升降轴上升",
            ArmTo(
                endpoint_xy=compiler.kin.forward(client.state.q1, client.state.q2).endpoint,
                h=0.38,
                gripper_yaw=client.state.gripper_yaw,
                gripper_opening=client.state.gripper_opening,
                speed_scale=0.5,
            ),
        )

        _run(
            compiler,
            executor,
            "升降轴下降",
            ArmTo(
                endpoint_xy=compiler.kin.forward(client.state.q1, client.state.q2).endpoint,
                h=0.24,
                gripper_yaw=client.state.gripper_yaw,
                gripper_opening=client.state.gripper_opening,
                speed_scale=0.5,
            ),
        )
    finally:
        client.close()


def _run(compiler: MotionCompiler, executor: MotionExecutor, name: str, motion) -> None:
    """编译、执行并打印一段联调动作。"""

    compiled = compiler.compile(executor.client.state, motion)
    print(f"{name}: points={compiled.stream.point_count} duration={compiled.duration:.3f}s")
    ok = executor.run_compiled(compiled, timeout_s=max(2.0, compiled.duration + 1.0))
    print(f"{name}: ok={ok} state={executor.client.state}")
    if not ok:
        raise RuntimeError(f"{name} 未收到到位反馈。")


if __name__ == "__main__":
    main()
