from __future__ import annotations

import time
from collections import deque
from dataclasses import dataclass
from typing import Callable

from connection.client import Client, MachineState
from plan import AbstractNode, ActionNode, DAG, WaitNode, children, dep_left

from .mixer import RunningAction, RuntimeMixer

DEFAULT_CONTROL_HZ = 100.0


class ExecutionError(RuntimeError):
    """DAG 执行失败。"""


@dataclass(frozen=True, slots=True)
class ExecutionResult:
    """一次 DAG 执行结果。"""

    success: bool
    completed_nodes: int
    final_state: MachineState


@dataclass(frozen=True, slots=True)
class RunningWait:
    """已启动 Wait 的运行时状态。"""

    node: WaitNode
    start_time: float


class MissionExecutor:
    """执行已经规划完成的 Plan DAG。"""

    def __init__(
        self,
        client: Client,
        *,
        control_hz: float = DEFAULT_CONTROL_HZ,
        clock: Callable[[], float] = time.perf_counter,
        sleep: Callable[[float], None] = time.sleep,
        mixer: RuntimeMixer | None = None,
    ) -> None:
        if control_hz <= 0.0:
            raise ValueError("control_hz 必须大于 0。")
        self.client = client
        self.control_hz = control_hz
        self.clock = clock
        self.sleep = sleep
        self.mixer = mixer or RuntimeMixer()

    def run(self, dag: DAG) -> ExecutionResult:
        """执行 DAG，失败时抛出 ExecutionError。"""

        child_map = children(dag.nodes)
        remaining_deps = dep_left(dag.nodes)
        ready = deque(node for node in dag.nodes if remaining_deps[node] == 0)
        active_actions: dict[ActionNode, RunningAction] = {}
        active_waits: dict[WaitNode, RunningWait] = {}
        done: set[AbstractNode] = set()
        kind_busy: dict[str, ActionNode] = {}
        hold_state = self.client.state
        period = 1.0 / self.control_hz
        next_tick = self.clock()

        def finish_node(node: AbstractNode) -> None:
            if node in done:
                raise ExecutionError(f"节点重复完成：{node.name}")
            done.add(node)
            for child in child_map[node]:
                remaining_deps[child] -= 1
                if remaining_deps[child] < 0:
                    raise ExecutionError(f"节点 {child.name} 依赖计数小于 0。")
                if remaining_deps[child] == 0:
                    ready.append(child)

        while len(done) < len(dag.nodes):
            now = self.clock()
            if self.client.error is not None:
                raise ExecutionError(f"通信接收失败：{self.client.error}") from self.client.error
            feedback = self.client.feedback

            self._update_waits(active_waits, feedback, now, finish_node)
            hold_state = self._update_actions(active_actions, kind_busy, hold_state, now, finish_node)
            self._start_ready_nodes(ready, active_actions, active_waits, kind_busy, now, finish_node)

            hold_state = self.mixer.mix(active_actions, hold_state, now)
            self.client.send_command(hold_state)

            next_tick += period
            sleep_s = next_tick - self.clock()
            if sleep_s > 0.0:
                self.sleep(sleep_s)

        return ExecutionResult(True, len(done), self.client.state)

    def _update_waits(
        self,
        active_waits: dict[WaitNode, RunningWait],
        feedback,
        now: float,
        finish_node,
    ) -> None:
        for node, running in list(active_waits.items()):
            if now - running.start_time > node.timeout:
                raise ExecutionError(f"WaitNode {node.name} 超时。")
            if feedback is None:
                continue
            if node.target is None:
                raise ExecutionError(f"WaitNode {node.name} 缺少 target。")
            if node.target.satisfied(feedback):
                del active_waits[node]
                finish_node(node)

    def _update_actions(
        self,
        active_actions: dict[ActionNode, RunningAction],
        kind_busy: dict[str, ActionNode],
        hold_state: MachineState,
        now: float,
        finish_node,
    ) -> MachineState:
        for node, running in list(active_actions.items()):
            path = node.path
            if path is None:
                raise ExecutionError(f"ActionNode {node.name} 缺少 path。")
            if now - running.start_time >= float(path.duration):
                end_time = running.start_time + float(path.duration)
                hold_state = self.mixer.mix({node: running}, hold_state, end_time)
                del active_actions[node]
                if kind_busy.get(node.kind) is not node:
                    raise ExecutionError(f"kind 锁状态不一致：{node.kind}")
                del kind_busy[node.kind]
                finish_node(node)
        return hold_state

    def _start_ready_nodes(
        self,
        ready: deque[AbstractNode],
        active_actions: dict[ActionNode, RunningAction],
        active_waits: dict[WaitNode, RunningWait],
        kind_busy: dict[str, ActionNode],
        now: float,
        finish_node,
    ) -> None:
        deferred: deque[AbstractNode] = deque()
        while ready:
            node = ready.popleft()
            if type(node) is AbstractNode:
                finish_node(node)
                continue
            if isinstance(node, WaitNode):
                if node in active_waits:
                    raise ExecutionError(f"WaitNode 重复启动：{node.name}")
                active_waits[node] = RunningWait(node, now)
                continue
            if isinstance(node, ActionNode):
                if node.kind in kind_busy:
                    deferred.append(node)
                    continue
                if node in active_actions:
                    raise ExecutionError(f"ActionNode 重复启动：{node.name}")
                active_actions[node] = RunningAction(node, now)
                kind_busy[node.kind] = node
                continue
            raise ExecutionError(f"未知节点类型：{node!r}")
        ready.extend(deferred)
