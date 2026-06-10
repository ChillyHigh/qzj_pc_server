from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True, slots=True, kw_only=True)
class Waypoint:
    """TOPPRA 输入航点。

    Args:
        speed_scale: 该航点附近的速度倍率，1.0 表示使用默认速度上限。
        meta: 透传给采样点的业务信息，例如 action_id。
        source_kind: 航点来源类型，用于判断连接处是否需要倒角。
        source_id: 同类来源的编号；同 kind 但 id 变化也视为新线段。
        blend_single: 单点航点是否允许被后续平滑处理。
    """

    speed_scale: float = 1.0
    meta: dict[str, Any] = field(default_factory=dict)
    source_kind: str = "single"
    source_id: int | None = None
    blend_single: bool = False

    @property
    def q(self) -> tuple[float, ...]:
        raise NotImplementedError


class TrajectoryError(ValueError):
    """轨迹生成失败、约束不一致或路径不可行。"""
