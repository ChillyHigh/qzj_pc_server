from __future__ import annotations

import numpy as np
from toppra.interpolator import AbstractGeometricPath

from trajectory import ToppraPlanner, Waypoint

from . import config

Pose = tuple[float, float, float]


class ChassisPathError(ValueError):
    """底盘 path 生成失败。"""


def direct(start: Pose, end: Pose, speed_scale: float) -> AbstractGeometricPath:
    """底盘直接从 start 移动到 end。"""

    waypoints = [
        Waypoint(q=start, speed_scale=speed_scale, source_kind="segment", source_id=0),
        Waypoint(q=end, speed_scale=speed_scale, source_kind="segment", source_id=0),
    ]
    return _planner().plan(waypoints)


def s_cross(start: Pose, end: Pose, speed_scale: float) -> AbstractGeometricPath:
    """左下到右上的 S 型跨场路径。"""

    if not (start[0] < -1.0 and start[1] < 0.0 and end[0] > 1.0 and end[1] > 0.0):
        raise ChassisPathError("s_cross 只接受左下到右上的跨场输入。")

    points = [(start[0], start[1]), *config.S_CROSS_POINTS, (end[0], end[1])]
    yaws = np.linspace(start[2], end[2], len(points))
    waypoints = [
        Waypoint(
            q=(float(point[0]), float(point[1]), float(yaws[idx])),
            speed_scale=speed_scale,
            source_kind="segment",
            source_id=idx,
        )
        for idx, point in enumerate(points)
    ]
    return _planner().plan(waypoints)


def _planner() -> ToppraPlanner:
    return ToppraPlanner(config.V_LIMIT, config.A_LIMIT)
