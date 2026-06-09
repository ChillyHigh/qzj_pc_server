from __future__ import annotations

import time
from random import shuffle

from connection import Client, WebSocketConfig, WebSocketTransport, SerialTransport, SerialConfig
from executor import MissionExecutor
from planner import Planner

import serial.tools.list_ports

FEEDBACK_WAIT_TIMEOUT_S = 2.0


def main() -> None:
    """使用 Planner 生成比赛 DAG 并执行。

    示例抽签：
      - 取货位 1 黄豆(1), 取货位 2 绿豆(2), 取货位 3 白芸豆(3)
      - 放置位 4→箱4, 5→箱1, 6→箱2, 7→箱3, 8→箱5
    即：黄豆→5, 绿豆→6, 白芸豆→7
    """

    pickup_assignment = [1, 2, 3]
    drop_assignment = [1, 2, 3, 4, 5]
    shuffle(drop_assignment)
    # drop_assignment = [2, 3, 4, 1, 5]
    print(pickup_assignment)
    print(drop_assignment)

    client = Client(WebSocketTransport(WebSocketConfig(url="ws://127.0.0.1:8765")))

    # 获取所有可用的串口
    ports = list(serial.tools.list_ports.comports())
    # 打印每个串口的描述信息
    for port in ports:
        print(port.device, port.name, port.description)

    usb_port = None

    for port in serial.tools.list_ports.comports():
        if "USB" in port.description:
            usb_port = port.device
            break

    print("使用串口:", usb_port)

    client = Client(SerialTransport(SerialConfig(
        port=usb_port,
        baud=230400,
        timeout=0.5,
    )))
    if not client.connect():
        raise SystemExit("无法连接通信后端。")

    try:
        initial_chassis, initial_arm = _read_initial_state(client)
        total_start = time.perf_counter()

        planner = Planner(initial_chassis, initial_arm)
        plan_start = time.perf_counter()
        dag, estimated_runtime = planner.generate(pickup_assignment, drop_assignment)
        plan_elapsed = time.perf_counter() - plan_start
        print(f"规划用时：{plan_elapsed:.3f}s")
        print(f"预计运行时间：{estimated_runtime:.3f}s")

        from debug import draw_dag
        draw_dag(dag, "dag.png")

        print(f"生成 DAG，共 {len(dag.nodes)} 个节点")
        execute_start = time.perf_counter()
        result = MissionExecutor(client, control_hz=100.0).run(dag)
        execute_elapsed = time.perf_counter() - execute_start
        total_elapsed = time.perf_counter() - total_start
        print(f"执行用时：{execute_elapsed:.3f}s")
        print(f"总用时：{total_elapsed:.3f}s")
        print(f"ok={result.success} completed={result.completed_nodes}")

    finally:
        time.sleep(0.05)
        client.close()


def _read_initial_state(
    client: Client,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float, float]]:
    """从反馈帧读取当前 chassis 和 arm Cartesian 状态。"""
    import arm

    # deadline = time.perf_counter() + FEEDBACK_WAIT_TIMEOUT_S
    while client.feedback is None:
        if client.error is not None:
            raise RuntimeError(f"通信接收失败：{client.error}") from client.error
        print(f"等待初始反馈，CRC错误数{client.feedback_crc_drop_count}")
        time.sleep(0.1)
    feedback = client.feedback
    if feedback is None:
        raise RuntimeError("未收到仿真反馈帧，不能确定 arm 起点。")

    kin = arm.FiveBarKinematics()
    ax, ay, ayaw = kin.fk(feedback.q1, feedback.q2, 0.0)
    chassis_state = (feedback.x, feedback.y, feedback.yaw)
    arm_state = (ax, ay, ayaw, feedback.h, client.state.gripper_opening)
    return chassis_state, arm_state


if __name__ == "__main__":
    main()
