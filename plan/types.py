from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable, Literal, Protocol

import numpy as np

ActionKind = Literal["chassis", "arm", "flags"]


@dataclass(frozen=True, slots=True)
class PlannedPath:
    """planner ↔ executor 路径契约。"""

    _sampler: Callable[[np.ndarray, int], np.ndarray] = field(repr=False)
    duration: float

    def __call__(self, path_positions, order: int = 0) -> np.ndarray:
        return self._sampler(np.asarray(path_positions, dtype=float), order)


class FeedbackTarget(Protocol):
    """WaitNode 使用的反馈判断契约。"""

    def satisfied(self, feedback: object) -> bool:
        """反馈满足目标时返回 True。"""


@dataclass(eq=False, slots=True)
class AbstractNode:
    """DAG 节点基类；依赖直接保存节点引用。"""

    name: str
    deps: list[AbstractNode] = field(default_factory=list)


@dataclass(eq=False, slots=True)
class ActionNode(AbstractNode):
    """一段占用单个 kind 资源的可采样轨迹。"""

    kind: ActionKind = "chassis"
    path: PlannedPath | None = None


@dataclass(eq=False, slots=True)
class WaitNode(AbstractNode):
    """只由反馈推进的门控节点。"""

    target: FeedbackTarget | None = None
    timeout: float = 0.0


@dataclass(eq=False, slots=True)
class DelayNode(AbstractNode):
    """只由时间推进的延时节点。"""

    duration: float = 0.0
