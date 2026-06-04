from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True, slots=True)
class DirectMove:
    """底盘直达目标姿态。

    `target` 为 `(x, y, yaw)`，单位 m/m/rad。
    """

    target: tuple[float, float, float]
    speed_scale: float = 1.0


@dataclass(frozen=True, slots=True)
class SMove:
    """底盘绕障碍 S 型移动。

    `target` 为 `(x, y, yaw)`；`lane_y` 是中间绕障通道的 y 坐标。
    """

    target: tuple[float, float, float]
    lane_y: float = 0.62
    speed_scale: float = 1.0


@dataclass(frozen=True, slots=True)
class ArmTo:
    """arm 末端直线移动到目标点。

    `endpoint_xy` 是机器人局部平面末端点；`h/gripper_yaw/gripper_opening`
    是本动作结束时的目标工具状态。
    """

    endpoint_xy: tuple[float, float]
    h: float
    gripper_yaw: float
    gripper_opening: float
    speed_scale: float = 1.0


@dataclass(frozen=True, slots=True)
class GripLift:
    """同步执行上升和合爪。

    保持当前末端 xy 与 `q1/q2`，只改变 `h` 和 `gripper_opening`。
    """

    lift_h: float
    closing: float
    speed_scale: float = 0.7


@dataclass(frozen=True, slots=True)
class MoveAndArm:
    """底盘和 arm 同步预备动作。

    只用于安全位或预备位，不用于夹爪伸入箱体的精确取放动作。
    """

    target_pose: tuple[float, float, float]
    endpoint_xy: tuple[float, float]
    h: float
    gripper_yaw: float
    gripper_opening: float
    speed_scale: float = 1.0


PresetMotion = DirectMove | SMove | ArmTo | GripLift | MoveAndArm

