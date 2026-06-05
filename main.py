from __future__ import annotations

import time

import arm
import funnel
from connection import Client, MachineState, WebSocketConfig, WebSocketTransport
from executor import MissionExecutor
from plan import AbstractNode, ActionNode, DAG

FEEDBACK_WAIT_TIMEOUT_S = 2.0


def main() -> None:
    """手写 DAG：测试五连杆跨越半平面。"""

    client = Client(WebSocketTransport(WebSocketConfig(url="ws://127.0.0.1:8765")))
    if not client.connect():
        raise SystemExit("无法连接通信后端。")

    try:
        start_state = _sync_state_from_feedback(client)
        kin = arm.FiveBarKinematics()
        start_x, start_y, _ = kin.fk(start_state.q1, start_state.q2, start_state.gripper_yaw)
        start_xy = (start_x, start_y)
        end_xy = (-start_xy[0], start_xy[1])
        start = AbstractNode("start")
        print(f"arm_cross: start_xy={start_xy} end_xy={end_xy}")

        go_to_start = ActionNode(
            name="arm_go_to_start",
            deps=[start],
            kind="arm",
            path=arm.move(
                (start_xy[0], start_xy[1], 0.0, start_state.h, start_state.gripper_opening),
                (0.2, 0.2, 0.0, start_state.h, start_state.gripper_opening),
                0.5,
            ),
        )

        arm_cross = ActionNode(
            name="arm_cross_half_plane",
            deps=[go_to_start],
            kind="arm",
            path=arm.move(
                (0.2, 0.2, 0.0, start_state.h, start_state.gripper_opening),
                (-0.3, 0.1, 0.0, start_state.h, start_state.gripper_opening),
                0.5,
            ),
        )

        open_funnel = ActionNode(
            "open_funnel",
            deps=[go_to_start],
            kind="flags",
            path=funnel.upper(True),
        )

        dag = DAG([start, go_to_start, arm_cross, open_funnel])
        result = MissionExecutor(client, control_hz=100.0).run(dag)
        print(f"ok={result.success} completed={result.completed_nodes} state={result.final_state}")
    finally:
        time.sleep(0.05)
        client.close()


def _sync_state_from_feedback(client: Client) -> MachineState:
    deadline = time.perf_counter() + FEEDBACK_WAIT_TIMEOUT_S
    while client.feedback is None and time.perf_counter() < deadline:
        if client.error is not None:
            raise RuntimeError(f"通信接收失败：{client.error}") from client.error
        time.sleep(0.01)
    feedback = client.feedback
    if feedback is None:
        raise RuntimeError("未收到仿真反馈帧，不能确定五连杆真实起点。")
    state = MachineState(
        x=feedback.x,
        y=feedback.y,
        yaw=feedback.yaw,
        h=feedback.h,
        q1=feedback.q1,
        q2=feedback.q2,
        gripper_yaw=client.state.gripper_yaw,
        gripper_opening=client.state.gripper_opening,
        flags=client.state.flags,
    )
    client.state = state
    return state


if __name__ == "__main__":
    main()
