from __future__ import annotations

from toppra.interpolator import AbstractGeometricPath

from trajectory import Waypoint

from . import config
from .five_bar import FiveBarKinematics
from .toppra_planner import ArmPathError, plan_joint_waypoints

# ArmCartesianState is expressed in the arm-local/chassis-aligned frame:
# (x, y, gripper_yaw, h, gripper_opening). gripper_yaw is relative to the
# chassis, while command q[3]=0 points opposite to the lower passive link.
# x/y are end-effector coordinates, not IK results.
ArmCartesianState = tuple[float, float, float, float, float]
EndEffectorState = ArmCartesianState
ArmJointState = tuple[float, float, float, float, float]


def move(start: EndEffectorState, end: EndEffectorState, speed_scale: float) -> AbstractGeometricPath:
    """末端从 start 移动到 end。

    start/end 是 arm 局部末端笛卡尔状态，gripper_yaw 相对底盘，
    `(x, y, gripper_yaw, h, gripper_opening)`，不是 IK 后的 q。
    跨 x=0 半平面时插入经验点。
    """

    kin = FiveBarKinematics()
    waypoints: list[Waypoint] = []
    q1, q2, gripper_yaw = kin.ik(start[0], start[1], start[2])
    waypoints.append(
        Waypoint(
            q=(
                start[3],
                q1,
                q2,
                gripper_yaw,
                start[4],
            ),
            speed_scale=speed_scale,
            source_kind="point",
            source_id=0,
        )
    )

    waypoints.extend(_half_plane_guide_waypoints(start, end, start[3], speed_scale, 2))

    q1, q2, gripper_yaw = kin.ik(end[0], end[1], end[2])
    waypoints.append(
        Waypoint(
            q=(
                end[3],
                q1,
                q2,
                gripper_yaw,
                end[4],
            ),
            speed_scale=speed_scale,
            source_kind="point",
            source_id=2,
        )
    )
    return plan_joint_waypoints(waypoints)


def prepare_pick(
    start: EndEffectorState,
    target: EndEffectorState,
    moving_h: float,
    speed_scale: float,
) -> AbstractGeometricPath:
    """先到移动高度，再移动到目标上方，最后下降到准备取货高度。

    start/target 是 arm 局部末端笛卡尔状态，gripper_yaw 相对底盘，
    `(x, y, gripper_yaw, h, gripper_opening)`，不是 IK 后的 q。
    """

    kin = FiveBarKinematics()
    waypoints: list[Waypoint] = []
    moving_start: EndEffectorState = (start[0], start[1], start[2], moving_h, start[4])
    moving_target: EndEffectorState = (target[0], target[1], target[2], moving_h, target[4])

    for state in (start, moving_start):
        q1, q2, gripper_yaw = kin.ik(state[0], state[1], state[2])
        waypoints.append(
            Waypoint(
                q=(
                    state[3],
                    q1,
                    q2,
                    gripper_yaw,
                    state[4],
                ),
                speed_scale=speed_scale,
                source_kind="segment",
                source_id=0,
            )
        )

    waypoints.extend(_half_plane_guide_waypoints(moving_start, moving_target, moving_h, speed_scale, 1))

    for state in (moving_target, target):
        q1, q2, gripper_yaw = kin.ik(state[0], state[1], state[2])
        waypoints.append(
            Waypoint(
                q=(
                    state[3],
                    q1,
                    q2,
                    gripper_yaw,
                    state[4],
                ),
                speed_scale=speed_scale,
                source_kind="segment",
                source_id=2,
            )
        )
    return plan_joint_waypoints(waypoints)


def set_gripper(state: EndEffectorState, opening: float) -> AbstractGeometricPath:
    """保持末端位姿，只改变夹爪开合角。

    state 是 arm 局部末端笛卡尔状态
    `(x, y, gripper_yaw, h, gripper_opening)`，opening 是目标开合角。
    """

    kin = FiveBarKinematics()
    q1, q2, gripper_yaw = kin.ik(state[0], state[1], state[2])
    waypoints = [
        Waypoint(
            q=(
                state[3],
                q1,
                q2,
                gripper_yaw,
                state[4],
            ),
            speed_scale=1.0,
            source_kind="point",
            source_id=0,
        ),
        Waypoint(
            q=(
                state[3],
                q1,
                q2,
                gripper_yaw,
                opening,
            ),
            speed_scale=1.0,
            source_kind="point",
            source_id=0,
        ),
    ]
    return plan_joint_waypoints(waypoints)


def _half_plane_guide_waypoints(
    start: EndEffectorState,
    end: EndEffectorState,
    h: float,
    speed_scale: float,
    source_id: int,
) -> list[Waypoint]:
    if start[0] * end[0] >= 0.0:
        return []

    waypoints: list[Waypoint] = []
    joint_states = config.HALF_PLANE_JOINT_STATES_NEG_TO_POSITIVE_X
    if start[0] > 0.0 and end[0] < 0.0:
        joint_states = tuple(reversed(joint_states))
    for q1, q2, gripper_yaw in joint_states:
        opening = start[4] if abs(q1 - joint_states[0][0]) < 1e-9 else end[4]
        waypoints.append(
            Waypoint(
                q=(
                    h,
                    q1,
                    q2,
                    gripper_yaw,
                    opening,
                ),
                speed_scale=speed_scale,
                source_kind="segment",
                source_id=source_id,
            )
        )
    return waypoints


def grip_lift(start: ArmJointState, end_opening: float, speed_scale: float) -> AbstractGeometricPath:
    """保持 q1/q2/gripper_yaw，同步上升 h 并改变 opening。

    start 是 joint-space 状态 `(h, q1, q2, gripper_yaw, gripper_opening)`。
    """

    end_h = start[0] + config.GRIP_LIFT_H
    if end_h > config.H_MAX:
        raise ArmPathError(f"grip_lift 目标 h 超出上限：{end_h:.4f}")
    waypoints = [
        Waypoint(q=start, speed_scale=speed_scale, source_kind="segment", source_id=0),
        Waypoint(
            q=(end_h, start[1], start[2], start[3], end_opening),
            speed_scale=speed_scale,
            source_kind="segment",
            source_id=0,
        ),
    ]
    return plan_joint_waypoints(waypoints)
