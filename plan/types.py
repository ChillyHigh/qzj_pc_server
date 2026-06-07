from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Literal, Protocol

if TYPE_CHECKING:
    from toppra.interpolator import AbstractGeometricPath

ActionKind = Literal["chassis", "arm", "flags"]


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
    path: AbstractGeometricPath | None = None


@dataclass(eq=False, slots=True)
class WaitNode(AbstractNode):
    """只由反馈推进的门控节点。"""

    target: FeedbackTarget | None = None
    timeout: float = 0.0


@dataclass(eq=False, slots=True)
class DelayNode(AbstractNode):
    """只由时间推进的延时节点。"""

    duration: float = 0.0
