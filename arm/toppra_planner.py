from __future__ import annotations

from toppra.interpolator import AbstractGeometricPath

from trajectory import ToppraPlanner, Waypoint

from . import config


class ArmPathError(ValueError):
    """arm path 生成失败。"""


def plan_joint_waypoints(waypoints: list[Waypoint]) -> AbstractGeometricPath:
    """对已经 IK 后的 arm joint-space 航点做 TOPPRA。"""

    planner = ToppraPlanner(
        config.V_LIMIT,
        config.A_LIMIT,
        q_min=config.Q_MIN_LIMIT,
        q_max=config.Q_MAX_LIMIT,
    )
    return planner.plan(waypoints)
