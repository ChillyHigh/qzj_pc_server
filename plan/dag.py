from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from .types import AbstractNode, ActionNode, DelayNode, WaitNode

KIND_DIMS = {
    "chassis": 3,
    "arm": 5,
    "flags": 1,
}


class DAGError(ValueError):
    """Plan DAG 契约不满足执行层要求。"""


@dataclass(frozen=True, slots=True)
class DAG:
    """执行层接收的动作 DAG。"""

    nodes: list[AbstractNode]

    def __post_init__(self) -> None:
        validate_dag(self.nodes)


def children(nodes: list[AbstractNode]) -> dict[AbstractNode, list[AbstractNode]]:
    """构建 node -> children 映射。"""

    result = {node: [] for node in nodes}
    for node in nodes:
        for dep in node.deps:
            result[dep].append(node)
    return result


def dep_left(nodes: list[AbstractNode]) -> dict[AbstractNode, int]:
    """构建 node -> 未完成依赖数映射。"""

    return {node: len(node.deps) for node in nodes}


def validate_dag(nodes: list[AbstractNode]) -> None:
    """校验 DAG 的结构和执行层可采样契约。"""

    if not nodes:
        raise DAGError("DAG 至少需要一个节点。")
    if len(set(nodes)) != len(nodes):
        raise DAGError("DAG 包含重复节点引用。")
    names = [node.name for node in nodes]
    if len(set(names)) != len(names):
        raise DAGError("DAG 节点 name 必须唯一，便于日志和报错。")

    node_set = set(nodes)
    for node in nodes:
        for dep in node.deps:
            if dep not in node_set:
                raise DAGError(f"节点 {node.name} 依赖了 DAG 外节点 {dep.name}。")
        if isinstance(node, ActionNode):
            _validate_action(node)
        elif isinstance(node, WaitNode):
            _validate_wait(node)
        elif isinstance(node, DelayNode):
            _validate_delay(node)
        elif type(node) is not AbstractNode:
            raise DAGError(f"未知节点类型：{node!r}")

    _validate_acyclic(nodes)


def _validate_action(node: ActionNode) -> None:
    if node.kind not in KIND_DIMS:
        raise DAGError(f"ActionNode {node.name} kind 非法：{node.kind}")
    if node.path is None:
        raise DAGError(f"ActionNode {node.name} 缺少 path。")
    duration = float(node.path.duration)
    if not np.isfinite(duration) or duration < 0.0:
        raise DAGError(f"ActionNode {node.name} path.duration 非法：{duration}")
    q0 = np.asarray(node.path(0.0, order=0), dtype=float)
    dq0 = np.asarray(node.path(0.0, order=1), dtype=float)
    expected_dim = KIND_DIMS[node.kind]
    if q0.shape != (expected_dim,):
        raise DAGError(f"ActionNode {node.name} q 维度应为 {expected_dim}，实际为 {q0.shape}。")
    if dq0.shape != (expected_dim,):
        raise DAGError(f"ActionNode {node.name} dq 维度应为 {expected_dim}，实际为 {dq0.shape}。")


def _validate_wait(node: WaitNode) -> None:
    if node.target is None:
        raise DAGError(f"WaitNode {node.name} 缺少 target。")
    if not hasattr(node.target, "satisfied"):
        raise DAGError(f"WaitNode {node.name} target 缺少 satisfied(feedback)。")
    timeout = float(node.timeout)
    if not np.isfinite(timeout) or timeout < 0.0:
        raise DAGError(f"WaitNode {node.name} timeout 不能为负数。")
    # timeout == 0 表示不限制等待时间


def _validate_delay(node: DelayNode) -> None:
    duration = float(node.duration)
    if not np.isfinite(duration) or duration < 0.0:
        raise DAGError(f"DelayNode {node.name} duration 不能为负数。")


def _validate_acyclic(nodes: list[AbstractNode]) -> None:
    remaining = dep_left(nodes)
    queue = deque(node for node in nodes if remaining[node] == 0)
    seen = 0
    child_map = children(nodes)
    while queue:
        node = queue.popleft()
        seen += 1
        for child in child_map[node]:
            remaining[child] -= 1
            if remaining[child] == 0:
                queue.append(child)
    if seen != len(nodes):
        raise DAGError("DAG 中存在环。")
