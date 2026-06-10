from __future__ import annotations

import math
import time
from collections.abc import Callable

import serial.tools.list_ports

import chassis
import funnel
from chassis.geometry import validate_chassis_path
from connection import Client, SerialConfig, SerialTransport, WebSocketTransport, WebSocketConfig
from executor import MissionExecutor
from plan import AbstractNode, ActionNode, DAG, DelayNode

from main import _read_initial_state

CONTROL_HZ = 100.0
SERIAL_BAUD = 230400
SERIAL_TIMEOUT_S = 0.5

# 改这个名字来选择本次下发哪个 DAG。
SELECTED_DAG = "chassis_forward_0_5m"

DrivePose = tuple[float, float, float]
ArmCartesian = tuple[float, float, float, float, float]
DagBuilder = Callable[[DrivePose, ArmCartesian], DAG]


def main() -> None:

    client = Client(WebSocketTransport(WebSocketConfig(url="ws://127.0.0.1:8765")))

    # client = Client(
    #     SerialTransport(
    #         SerialConfig(
    #             port=_find_usb_serial_port(),
    #             baud=SERIAL_BAUD,
    #             timeout=SERIAL_TIMEOUT_S,
    #         )
    #     )
    # )
    if not client.connect():
        raise SystemExit("无法连接通信后端。")

    try:
        initial_chassis, initial_arm = _read_initial_state(client)

        start = AbstractNode(f"{SELECTED_DAG}_start")

        target = _relative_chassis_target(initial_chassis, forward_m=0.5)

        move_forward = ActionNode(
            name="move_forward",
            deps=[start],
            kind="chassis",
            path=chassis.direct(initial_chassis, target),
        )

        wait = DelayNode(name="wait", deps=[move_forward], duration=1.0)

        move_backward = ActionNode(
            name="move_backward",
            deps=[wait],
            kind="chassis",
            path=chassis.direct(target, initial_chassis),
        )

        dag = DAG([start, wait, move_forward, move_backward])

        print(f"初始底盘：x={initial_chassis[0]:.3f}, y={initial_chassis[1]:.3f}, yaw={initial_chassis[2]:.3f}")
        print(f"节点数：{len(dag.nodes)}")

        result = MissionExecutor(client, control_hz=CONTROL_HZ).run(dag)
        print(f"ok={result.success} completed={result.completed_nodes}")
    finally:
        time.sleep(0.05)
        client.close()


def _find_usb_serial_port() -> str:
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        print(port.device, port.name, port.description)

    for port in ports:
        if "USB" in port.description:
            print("使用串口:", port.device)
            return str(port.device)

    available = ", ".join(f"{p.device}({p.description})" for p in ports) or "无"
    raise RuntimeError(f"未找到 description 包含 USB 的串口。当前可用串口：{available}")

def _relative_chassis_target(start: DrivePose, forward_m: float = 0.0, left_m: float = 0.0, yaw_delta: float = 0.0) -> DrivePose:
    x, y, yaw = start
    dx = forward_m * math.cos(yaw) - left_m * math.sin(yaw)
    dy = forward_m * math.sin(yaw) + left_m * math.cos(yaw)
    return (x + dx, y + dy, yaw + yaw_delta)


if __name__ == "__main__":
    main()
