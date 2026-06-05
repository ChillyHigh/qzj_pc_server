from __future__ import annotations

import math
from typing import Literal


DropCarrier = Literal["upper_funnel", "lower_funnel", "gripper"]

Pose = tuple[float, float, float, float, float, float, float]
TargetRect = tuple[tuple[float, float], tuple[float, float]]

DRIVE_X = 0
DRIVE_Y = 1
DRIVE_YAW = 2
ARM_X = 3
ARM_Y = 4
GRIPPER_YAW = 5
H = 6

TARGET_CENTER = 0
TARGET_HALF_SIZE = 1

CHASSIS_CENTER_FROM_DRIVE = (0.035, 0.0)
CHASSIS_LENGTH = 0.360
CHASSIS_WIDTH = 0.670
ARM_ORIGIN_IN_CHASSIS_CENTER = (-0.110, 0.0)

OBSTACLE_RADIUS = 0.051
OBSTACLE_CENTERS = ((-1.000, 0.000), (1.000, 0.000))

PICKUP_H = 0.0
GRIPPER_DROP_H = 0.0
FUNNEL_POSE_H = 0.0

FUNNEL_EDGE_IN_CHASSIS_CENTER_X = 0.070
UPPER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y = 0.365
LOWER_FUNNEL_EDGE_IN_CHASSIS_CENTER_Y = -0.365

TARGET_RECTS: dict[int, TargetRect] = {
    1: ((-1.855, 0.500), (0.105, 0.150)),
    2: ((-1.855, -0.500), (0.105, 0.150)),
    3: ((-1.600, 0.000), (0.105, 0.150)),
    4: ((1.640, 0.875), (0.150, 0.105)),
    5: ((1.875, 0.400), (0.105, 0.150)),
    6: ((1.875, 0.000), (0.105, 0.150)),
    7: ((1.875, -0.400), (0.105, 0.150)),
    8: ((1.640, -0.875), (0.150, 0.105)),
}

PICKUP_POSES: dict[int, Pose] = {
    1: (-1.480, 0.500, 0.0, -0.300, 0.0, 0.0, PICKUP_H),
    2: (-1.480, -0.500, 0.0, -0.300, 0.0, 0.0, PICKUP_H),
    3: (-1.315, 0.000, 0.0, -0.210, 0.0, 0.0, PICKUP_H),
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
        "gripper": (1.374835, 0.609835, math.radians(225.0), -0.300, 0.0, math.radians(45.0), GRIPPER_DROP_H),
    },
    5: {
        "upper_funnel": (1.405, 0.505, -math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, 0.295, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, 0.400000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    6: {
        "upper_funnel": (1.405, 0.105, -math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, -0.105, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, 0.000000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    7: {
        "upper_funnel": (1.405, -0.295, -math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.405, -0.505, math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.500000, -0.400000, math.pi, -0.300, 0.0, 0.0, GRIPPER_DROP_H),
    },
    8: {
        "upper_funnel": (1.125, -0.770, -math.pi / 2.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "lower_funnel": (1.535, -0.405, 0.0, 0.0, 0.0, 0.0, FUNNEL_POSE_H),
        "gripper": (1.374835, -0.609835, math.radians(135.0), -0.300, 0.0, math.radians(135.0), GRIPPER_DROP_H),
    },
}
