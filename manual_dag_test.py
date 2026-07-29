from __future__ import annotations

import math
import time
from collections.abc import Callable
from dataclasses import replace

import serial.tools.list_ports

import arm
import chassis
import funnel
from chassis.geometry import validate_chassis_path
from connection import Client, SerialConfig, SerialTransport, WebSocketTransport, WebSocketConfig
from executor import MissionExecutor
from plan import ActionNode, DAG, DelayNode, StartNode

from main import _read_initial_state

CONTROL_HZ = 100.0
SERIAL_BAUD = 230400
SERIAL_TIMEOUT_S = 0.5
SERVO_LOW_HOLD_S = 1.0
SERVO_LOW_RAD = math.radians(30.0)
SERVO_HIGH_RAD = math.radians(150.0)

# 改这个名字来选择本次下发哪个 DAG。
SELECTED_DAG = "chassis_forward_0_5m"

DrivePose = tuple[float, float, float]
ArmCartesian = tuple[float, float, float, float, float]
DagBuilder = Callable[[DrivePose, ArmCartesian], DAG]


def main() -> None:

    # client = Client(WebSocketTransport(WebSocketConfig(url="ws://127.0.0.1:8765")))

    client = Client(
        SerialTransport(
            SerialConfig(
                port=_find_usb_serial_port(),
                baud=SERIAL_BAUD,
                timeout=SERIAL_TIMEOUT_S,
            )
        )
    )
    if not client.connect():
        raise SystemExit("无法连接通信后端。")

    # input("继续？")
    try:

        dag = get_servo_dag(client)
        # print(f"初始底盘：x={initial_chassis[0]:.3f}, y={initial_chassis[1]:.3f}, yaw={initial_chassis[2]:.3f}")
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


def get_servo_dag(client: Client) -> DAG:
    """保持其他机构当前位置，让两路舵机从 30 度运动到 150 度。"""

    initial_state = replace(
        _read_initial_state(client),
        gripper_yaw=SERVO_LOW_RAD,
        gripper_opening=SERVO_LOW_RAD,
    )
    start = StartNode("start", initial_state)
    hold_low = DelayNode(
        name="servo_low_hold",
        deps=[start],
        duration=SERVO_LOW_HOLD_S,
    )
    move = ActionNode(
        name="servo_low_to_high",
        deps=[hold_low],
        kind="arm",
        path=arm.ArmToppraPlanner().plan(
            [
                arm.ArmWaypoint(
                    h=initial_state.h,
                    q1=initial_state.q1,
                    q2=initial_state.q2,
                    gripper_yaw=SERVO_LOW_RAD,
                    gripper_opening=SERVO_LOW_RAD,
                ),
                arm.ArmWaypoint(
                    h=initial_state.h,
                    q1=initial_state.q1,
                    q2=initial_state.q2,
                    gripper_yaw=SERVO_HIGH_RAD,
                    gripper_opening=SERVO_HIGH_RAD,
                ),
            ]
        ),
    )
    return DAG([start, hold_low, move])


def get_dag_3(client: Client) -> DAG:
    initial_state = _read_initial_state(client)

    kin = arm.FiveBarKinematics()
    arm_x, arm_y, arm_yaw = kin.fk(initial_state.q1, initial_state.q2, 0.0)
    initial_arm: ArmCartesian = (
        arm_x,
        arm_y,
        arm_yaw,
        initial_state.h,
        initial_state.gripper_opening,
    )
    target_arm: ArmCartesian = (
        -arm_x,
        -arm_y,
        arm_yaw,
        initial_state.h,
        initial_state.gripper_opening,
    )

    start = StartNode("start", initial_state)
    move_to = ActionNode(
        name="arm_move_to_neg_xy",
        deps=[start],
        kind="arm",
        path=arm.move(initial_arm, target_arm),
    )
    move_back = ActionNode(
        name="arm_move_back_initial",
        deps=[move_to],
        kind="arm",
        path=arm.move(target_arm, initial_arm),
    )

    dag = DAG([start, move_to, move_back])
    return dag



if __name__ == "__main__":
    main()
