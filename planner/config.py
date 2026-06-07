from __future__ import annotations

import math
from typing import Literal

from plan.setting import (  # 场地物理常量（chassis 也可引用）
    ARM_ORIGIN_IN_CHASSIS_CENTER,
    ARM_X,
    ARM_Y,
    CHASSIS_CENTER_FROM_DRIVE,
    CHASSIS_HALF_X_FRONT,
    CHASSIS_HALF_X_REAR,
    CHASSIS_HALF_Y,
    CHASSIS_LENGTH,
    CHASSIS_WIDTH,
    DRIVE_X,
    DRIVE_Y,
    DRIVE_YAW,
    GRIPPER_YAW,
    H,
    OBSTACLE_CENTERS,
    OBSTACLE_RADIUS,
    TARGET_CENTER,
    TARGET_HALF_SIZE,
    TARGET_RECTS,
    TargetRect,
)

# ---- 比赛类型 ---------------------------------------------------------------

DropCarrier = Literal["upper_funnel", "lower_funnel", "gripper"]

Pose = tuple[float, float, float, float, float, float, float]

# ---- 高度常量 ---------------------------------------------------------------

PICKUP_H = 0.4
GRIPPER_DROP_H = 0.3
FUNNEL_POSE_H = 0.3

# ---- 漏斗几何 ---------------------------------------------------------------

FUNNEL_EDGE_IN_CHASSIS_CENTER_X = 0.070
UPPER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y = 0.365
LOWER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y = -0.365

# ---- 预设姿态 ---------------------------------------------------------------

豆子厚度 = 0.05

货箱高 = 0.15
#  x, y, yaw, x, y, g_y, h
PICKUP_POSES: dict[int, Pose] = {
    1: (-1.480, 0.500, 0.0, -0.300, 0.0, 0.0, 0.1),
    2: (-1.480, -0.500, 0.0, -0.300, 0.0, 0.0, 0.05),
    3: (-1.300, 0.000, 0.0, -0.225, 0.0, 0.0, 0.15),
}

FUNNEL_DROP_BOX_EDGE_POINTS: dict[int, dict[DropCarrier, tuple[float, float]]] = {
    4: {
        "upper_funnel": (1.640, 0.770),
        "lower_funnel": (1.490, 0.875),
    },
    5: {
        "upper_funnel": (1.770, 0.400),
        "lower_funnel": (1.770, 0.400),
    },
    6: {
        "upper_funnel": (1.770, 0.000),
        "lower_funnel": (1.770, 0.000),
    },
    7: {
        "upper_funnel": (1.770, -0.400),
        "lower_funnel": (1.770, -0.400),
    },
    8: {
        "upper_funnel": (1.490, -0.875),
        "lower_funnel": (1.640, -0.770),
    },
}

DROP_POSES: dict[int, dict[DropCarrier, Pose]] = {
    4: {
        "upper_funnel": (1.535, 0.405, 0.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.125, 0.770, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.360693, 0.595693, math.radians(225.0), -0.320, 0.0, math.radians(45.0), GRIPPER_DROP_H),
    },
    5: {
        "upper_funnel": (1.405, 0.505, 3.0 * math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, 0.295, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, 0.400000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    6: {
        "upper_funnel": (1.405, 0.105, 3.0 * math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, -0.105, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, 0.000000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    7: {
        "upper_funnel": (1.405, -0.295, 3.0 * math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, -0.505, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, -0.400000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    8: {
        "upper_funnel": (1.125, -0.770, 3.0 * math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.535, -0.405, 0.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.360693, -0.595693, math.radians(135.0), -0.320, 0.0, math.radians(135.0), GRIPPER_DROP_H),
    },
}

# ---- 运行时常量 ---------------------------------------------------------------

GRIPPER_OPEN_ANGLE = math.radians(150.0)
GRIPPER_CLOSED_ANGLE = 0.0

放漏斗高度 = 0.3

CHASSIS_SPEED_SCALE = 0.8
ARM_SPEED_SCALE = 0.8
ARM_FUNNEL_SPEED_SCALE = 0.5
S_CROSS_SPEED_SCALE = 0.8

PREPARE_PICK_MOVING_H = 0.42

WAIT_POS_TOLERANCE = 0.02
WAIT_YAW_TOLERANCE = math.radians(3.0)
WAIT_TIMEOUT = 0.0
WAIT_CROSS_TIMEOUT = 0.0

FINISH_DRIVE: tuple[float, float, float] = (0.3, 0.0, 0.0)

FUNNEL_ARM_TARGET: dict[str, tuple[float, float]] = {
    "upper_funnel": (0.180, 0.185),
    "lower_funnel": (0.180, -0.185),
}
FUNNEL_RELEASE_H = 0.20
