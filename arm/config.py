from __future__ import annotations

import math

from .types import FiveBarParams

H_MIN = 0.0
H_MAX = 0.43153
Q_MIN = 0.0
Q_MAX = 2.0 * math.pi
GRIPPER_YAW_MIN = 0.0
GRIPPER_YAW_MAX = 2.0 * math.pi
GRIPPER_OPENING_MIN = 0.0
GRIPPER_OPENING_MAX = math.radians(150.0)
IK_DISTANCE_MARGIN = 0.005

V_LIMIT = (
    0.45,
    math.radians(180.0),
    math.radians(180.0),
    math.radians(180.0),
    math.radians(220.0),
)
A_LIMIT = (
    1.0,
    math.radians(360.0),
    math.radians(360.0),
    math.radians(360.0),
    math.radians(440.0),
)
Q_MIN_LIMIT = (H_MIN, Q_MIN, Q_MIN, GRIPPER_YAW_MIN, GRIPPER_OPENING_MIN)
Q_MAX_LIMIT = (H_MAX, Q_MAX, Q_MAX, GRIPPER_YAW_MAX, GRIPPER_OPENING_MAX)

GRIP_LIFT_H = 0.130
HALF_PLANE_STEP_DEG = 0.5
HALF_PLANE_JOINT_POINTS_NEG_TO_POSITIVE_X = tuple(
    (math.radians(185.0 - idx * HALF_PLANE_STEP_DEG), math.radians(175.0 + idx * HALF_PLANE_STEP_DEG))
    for idx in range(int((185.0 - 175.0) / HALF_PLANE_STEP_DEG) + 1)
)
_FIVE_BAR_PARAMS = FiveBarParams()
_HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X = []
for q1, q2 in HALF_PLANE_JOINT_POINTS_NEG_TO_POSITIVE_X:
    lower_elbow_x = _FIVE_BAR_PARAMS.motor_x - _FIVE_BAR_PARAMS.active_link * math.sin(q2)
    lower_elbow_y = -_FIVE_BAR_PARAMS.motor_y_offset + _FIVE_BAR_PARAMS.active_link * math.cos(q2)
    dx_sq = _FIVE_BAR_PARAMS.passive_link * _FIVE_BAR_PARAMS.passive_link - lower_elbow_y * lower_elbow_y
    dx = math.sqrt(max(dx_sq, 0.0))
    endpoint_x = lower_elbow_x + dx
    if q1 > math.pi:
        endpoint_x = lower_elbow_x - dx
    lower_passive_yaw = math.atan2(-lower_elbow_y, endpoint_x - lower_elbow_x)
    _HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X.append(
        (q1, q2, (-lower_passive_yaw - math.pi) % (2.0 * math.pi))
    )
HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X = tuple(_HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X)
del _FIVE_BAR_PARAMS
del _HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X
