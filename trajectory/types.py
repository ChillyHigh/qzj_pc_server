from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np


@dataclass(frozen=True, slots=True)
class Waypoint:
    """TOPPRA 输入航点。

    Args:
        q: 广义坐标。底盘为 (x, y, yaw)，arm 为
            (h, q1, q2, gripper_yaw, gripper_opening)。
        speed_scale: 该航点附近的速度倍率，1.0 表示使用默认速度上限。
        meta: 透传给采样点的业务信息，例如 action_id。
        source_kind: 航点来源类型，用于判断连接处是否需要倒角。
        source_id: 同类来源的编号；同 kind 但 id 变化也视为新线段。
        blend_single: 单点航点是否允许被后续平滑处理。
    """

    q: tuple[float, ...]
    speed_scale: float = 1.0
    meta: dict[str, Any] = field(default_factory=dict)
    source_kind: str = "single"
    source_id: int | None = None
    blend_single: bool = False

class TrajectoryError(ValueError):
    """轨迹生成失败、约束不一致或路径不可行。"""
