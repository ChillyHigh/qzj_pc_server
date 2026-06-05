from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from connection.client import MachineState
from plan import ActionNode


@dataclass(frozen=True, slots=True)
class RunningAction:
    """已启动 Action 的运行时状态。"""

    node: ActionNode
    start_time: float


class RuntimeMixer:
    """把当前活跃 Action 合并成单帧控制状态。"""

    def mix(
        self,
        active_actions: Mapping[ActionNode, RunningAction],
        hold_state: MachineState,
        now: float,
    ) -> MachineState:
        state = _zero_velocities(hold_state)
        for running in active_actions.values():
            elapsed = now - running.start_time
            path = running.node.path
            if path is None:
                raise RuntimeError(f"ActionNode {running.node.name} 缺少 path。")
            sample_t = min(max(elapsed, 0.0), float(path.duration))
            q = np.asarray(path(sample_t, order=0), dtype=float)
            dq = np.asarray(path(sample_t, order=1), dtype=float)
            state = _apply_action_sample(state, running.node, q, dq)
        return state


def _zero_velocities(state: MachineState) -> MachineState:
    return MachineState(
        x=state.x,
        y=state.y,
        yaw=state.yaw,
        h=state.h,
        q1=state.q1,
        q2=state.q2,
        gripper_yaw=state.gripper_yaw,
        gripper_opening=state.gripper_opening,
        dx=0.0,
        dy=0.0,
        dyaw=0.0,
        dh=0.0,
        dq1=0.0,
        dq2=0.0,
        flags=state.flags,
    )


def _apply_action_sample(
    state: MachineState,
    node: ActionNode,
    q: np.ndarray,
    dq: np.ndarray,
) -> MachineState:
    if node.kind == "chassis":
        if q.shape != (3,) or dq.shape != (3,):
            raise RuntimeError(f"chassis action {node.name} 采样维度错误。")
        return MachineState(
            x=float(q[0]),
            y=float(q[1]),
            yaw=float(q[2]),
            h=state.h,
            q1=state.q1,
            q2=state.q2,
            gripper_yaw=state.gripper_yaw,
            gripper_opening=state.gripper_opening,
            dx=float(dq[0]),
            dy=float(dq[1]),
            dyaw=float(dq[2]),
            dh=state.dh,
            dq1=state.dq1,
            dq2=state.dq2,
            flags=state.flags,
        )
    if node.kind == "arm":
        if q.shape != (5,) or dq.shape != (5,):
            raise RuntimeError(f"arm action {node.name} 采样维度错误。")
        return MachineState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            h=float(q[0]),
            q1=float(q[1]),
            q2=float(q[2]),
            gripper_yaw=float(q[3]),
            gripper_opening=float(q[4]),
            dx=state.dx,
            dy=state.dy,
            dyaw=state.dyaw,
            dh=float(dq[0]),
            dq1=float(dq[1]),
            dq2=float(dq[2]),
            flags=state.flags,
        )
    if node.kind == "flags":
        if q.shape != (1,):
            raise RuntimeError(f"flags action {node.name} 采样维度错误。")
        return MachineState(
            x=state.x,
            y=state.y,
            yaw=state.yaw,
            h=state.h,
            q1=state.q1,
            q2=state.q2,
            gripper_yaw=state.gripper_yaw,
            gripper_opening=state.gripper_opening,
            dx=state.dx,
            dy=state.dy,
            dyaw=state.dyaw,
            dh=state.dh,
            dq1=state.dq1,
            dq2=state.dq2,
            flags=int(q[0]),
        )
    raise RuntimeError(f"未知 action kind：{node.kind}")
