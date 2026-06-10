from __future__ import annotations

import math

from plan.types import PlannedPath

from .geometry import plan_avoidance_path, validate_chassis_path
from .toppra_planner import ChassisPathError, ChassisToppraPlanner, ChassisWaypoint

Pose = tuple[float, float, float]


def _shortest_yaw_delta(from_yaw: float, to_yaw: float) -> float:
    """从 from_yaw 到 to_yaw 的最短角增量，π 平局时选择正方向。"""

    delta = (to_yaw - from_yaw + math.pi) % (2.0 * math.pi) - math.pi
    if math.isclose(delta, -math.pi, abs_tol=1e-9):
        return math.pi
    return delta


def _continuous_yaw_points(points: list[Pose]) -> list[Pose]:
    if not points:
        return []
    result = [points[0]]
    accum = points[0][2]
    for point in points[1:]:
        accum += _shortest_yaw_delta(accum, point[2])
        result.append((point[0], point[1], accum))
    return result


def _unwrap_end_yaw(start: Pose, end: Pose) -> Pose:
    """将 end 的 yaw 相对 start 解缠绕，使转动走最短角路径。"""

    dyaw = _shortest_yaw_delta(start[2], end[2])
    return (end[0], end[1], start[2] + dyaw)


def direct(start: Pose, end: Pose) -> PlannedPath:
    """底盘从 start 移动到 end，无避障。"""

    unwrapped_end = _unwrap_end_yaw(start, end)
    waypoints = [
        ChassisWaypoint(x=start[0], y=start[1], yaw=start[2], source_kind="segment", source_id=0),
        ChassisWaypoint(
            x=unwrapped_end[0],
            y=unwrapped_end[1],
            yaw=unwrapped_end[2],
            source_kind="segment",
            source_id=1,
        ),
    ]
    return _planner().plan(waypoints)


def move(start: Pose, end: Pose) -> PlannedPath:
    """底盘从 start 移动到 end，自动避障。"""

    try:
        validate_chassis_path(start, end)
        unwrapped_end = _unwrap_end_yaw(start, end)
        waypoints = [
            ChassisWaypoint(x=start[0], y=start[1], yaw=start[2], source_kind="segment", source_id=0),
            ChassisWaypoint(
                x=unwrapped_end[0],
                y=unwrapped_end[1],
                yaw=unwrapped_end[2],
                source_kind="segment",
                source_id=1,
            ),
        ]
    except ValueError:
        detour = _continuous_yaw_points(plan_avoidance_path(start, end))
        waypoints = [
            ChassisWaypoint(
                x=float(p[0]),
                y=float(p[1]),
                yaw=float(p[2]),
                source_kind="segment",
                source_id=idx,
            )
            for idx, p in enumerate(detour)
        ]
    return _planner().plan(waypoints)


def s_cross(start: Pose, end: Pose) -> PlannedPath:
    """左下到右上的跨场路径，经 (-0.5,-0.5) 和 (0.5, 0.5) 三段 Theta* 避障。"""

    if not (start[0] < -1.0 and start[1] < 0.0 and end[0] > 1.0):
        raise ChassisPathError("s_cross 只接受左下到右侧的跨场输入。")

    cp1: Pose = (-0.5, -0.5, start[2])
    cp2: Pose = (0.5, 0.5, end[2])

    seg1 = plan_avoidance_path(start, cp1)
    seg2 = [seg1[-1], cp2]
    seg3 = plan_avoidance_path(seg2[-1], end)
    all_points = _continuous_yaw_points(seg1 + seg2[1:] + seg3[1:])

    waypoints = [
        ChassisWaypoint(
            x=float(p[0]),
            y=float(p[1]),
            yaw=float(p[2]),
            source_kind="segment",
            source_id=0,
        )
        for p in all_points
    ]
    return _planner().plan(waypoints)


def _planner() -> ChassisToppraPlanner:
    return ChassisToppraPlanner()
