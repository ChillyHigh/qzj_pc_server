from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class FiveBarParams:
    """五连杆几何参数。

    arm path API 使用五连杆局部坐标：两电机中心线为 x=0，
    上下电机中点为 y=0。不要混用底盘或 MuJoCo 场景坐标。
    q1/q2 范围为 [0, 2pi]。上侧电机 q1=0 指向局部 -y；
    下侧电机 q2=0 指向局部 +y。
    """

    motor_x: float = 0.0
    motor_y_offset: float = 0.080
    active_link: float = 0.160
    passive_link: float = 0.240
    q_min: float = 0.0
    q_max: float = 2.0 * math.pi


class ArmKinematicsError(ValueError):
    """五连杆运动学无效或目标点不可达。"""
