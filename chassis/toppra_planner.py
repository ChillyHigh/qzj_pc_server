from __future__ import annotations

from dataclasses import dataclass

from toppra.interpolator import AbstractGeometricPath

from plan.types import PlannedPath
from trajectory import ToppraPlanner, Waypoint

from . import config


class ChassisPathError(ValueError):
    """底盘 path 生成失败。"""


@dataclass(frozen=True, slots=True)
class ChassisWaypoint(Waypoint):
    x: float
    y: float
    yaw: float

    @property
    def q(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


class ChassisToppraPlanner(ToppraPlanner):
    def __init__(self) -> None:
        super().__init__(vlim=config.V_LIMIT, alim=config.A_LIMIT)

    def plan(
        self,
        waypoints: list[ChassisWaypoint],
        *,
        max_step: float = 0.05,
    ) -> PlannedPath:
        trajectory: AbstractGeometricPath = super().plan(waypoints, max_step=max_step)
        return PlannedPath(trajectory, float(trajectory.duration))
