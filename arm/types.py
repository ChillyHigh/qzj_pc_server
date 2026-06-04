from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FiveBarParams:
    """五连杆几何参数。

    q1/q2 范围为 [-pi, pi]；q=0 时主动杆指向机器人局部 +y。
    """

    motor_x: float = 0.110
    motor_y_offset: float = 0.080
    active_link: float = 0.160
    passive_link: float = 0.240
    q_min: float = -math.pi
    q_max: float = math.pi


@dataclass(frozen=True, slots=True)
class ArmSolution:
    """五连杆一组完整构型解。

    endpoint 是夹爪末端在机器人局部平面内的位置，格式为 `(x, y)`。
    """

    q1: float
    q2: float
    upper_elbow: tuple[float, float]
    lower_elbow: tuple[float, float]
    endpoint: tuple[float, float]


class ArmKinematicsError(ValueError):
    """五连杆运动学无效或目标点不可达。"""
