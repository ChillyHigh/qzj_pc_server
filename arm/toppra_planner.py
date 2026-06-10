from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from toppra.interpolator import AbstractGeometricPath

from plan.types import PlannedPath
from trajectory import ToppraPlanner, TrajectoryError, Waypoint

from . import config


class ArmPathError(ValueError):
    """arm path 生成失败。"""


@dataclass(frozen=True, slots=True)
class ArmWaypoint(Waypoint):
    h: float
    q1: float
    q2: float
    gripper_yaw: float
    gripper_opening: float

    @property
    def q(self) -> tuple[float, float, float, float, float]:
        return (self.h, self.q1, self.q2, self.gripper_yaw, self.gripper_opening)


@dataclass(frozen=True, slots=True)
class _MotionWaypoint(Waypoint):
    h: float
    q1: float
    q2: float

    @property
    def q(self) -> tuple[float, float, float]:
        return (self.h, self.q1, self.q2)


class ArmToppraPlanner(ToppraPlanner):
    """Arm TOPPRA planner。

    h/q1/q2 参与 TOPPRA 速度和加速度约束；gripper_yaw/gripper_opening
    是舵机目标角，只按同一时间轴输出目标值，不生成速度前馈。
    """

    def __init__(self) -> None:
        super().__init__(
            vlim=config.MOTION_V_LIMIT,
            alim=config.MOTION_A_LIMIT,
            q_min=config.Q_MIN_LIMIT[:3],
            q_max=config.Q_MAX_LIMIT[:3],
        )

    def plan(
        self,
        waypoints: list[ArmWaypoint],
        *,
        max_step: float = 0.05,
    ) -> PlannedPath:
        if len(waypoints) < 2:
            raise TrajectoryError("Arm TOPPRA 至少需要 2 个航点。")

        motion_waypoints = [
            _MotionWaypoint(
                h=float(waypoint.h),
                q1=float(waypoint.q1),
                q2=float(waypoint.q2),
                speed_scale=waypoint.speed_scale,
                meta=dict(waypoint.meta),
                source_kind=waypoint.source_kind,
                source_id=waypoint.source_id,
                blend_single=waypoint.blend_single,
            )
            for waypoint in waypoints
        ]
        motion_qs = np.asarray([wp.q[:3] for wp in waypoints], dtype=float)
        servo_qs = np.asarray([wp.q[3:5] for wp in waypoints], dtype=float)
        servo_v_limit = np.asarray(
            (config.GRIPPER_YAW_SERVO_V_LIMIT, config.GRIPPER_OPENING_SERVO_V_LIMIT),
            dtype=float,
        )
        if np.allclose(motion_qs, motion_qs[0], atol=1e-9):
            return _create_arm_path(None, motion_qs, servo_qs, servo_v_limit)
        motion_path: AbstractGeometricPath = super().plan(motion_waypoints, max_step=max_step)
        return _create_arm_path(motion_path, motion_qs, servo_qs, servo_v_limit)


def plan_joint_waypoints(waypoints: list[ArmWaypoint]) -> PlannedPath:
    """对已经 IK 后的 arm joint-space 航点做 TOPPRA。"""

    return ArmToppraPlanner().plan(waypoints)


def _create_arm_path(
    motion_path: AbstractGeometricPath | None,
    motion_waypoints: np.ndarray,
    servo_waypoints: np.ndarray,
    servo_v_limit: np.ndarray,
) -> PlannedPath:
    motion_duration = 0.0 if motion_path is None else float(motion_path.duration)
    if motion_path is None:
        waypoint_times = np.zeros(len(motion_waypoints), dtype=float)
    else:
        t_grid = np.linspace(0.0, motion_duration, max(2, len(motion_waypoints) * 20))
        motion_samples = np.asarray(motion_path(t_grid, order=0), dtype=float)
        times = []
        for waypoint_q in motion_waypoints:
            nearest_idx = int(np.argmin(np.linalg.norm(motion_samples - waypoint_q, axis=1)))
            times.append(float(t_grid[nearest_idx]))
        waypoint_times = np.maximum.accumulate(np.asarray(times, dtype=float))
        waypoint_times[0] = 0.0
        waypoint_times[-1] = motion_duration

    completion = 0.0
    if len(servo_waypoints) >= 2:
        for dim in range(2):
            actual = float(servo_waypoints[0, dim])
            target = actual
            last_time = 0.0
            speed = float(servo_v_limit[dim])
            for idx in range(1, len(servo_waypoints)):
                issue_time = float(waypoint_times[idx - 1])
                actual = _move_towards(actual, target, speed, issue_time - last_time)
                target = float(servo_waypoints[idx, dim])
                last_time = issue_time
            completion = max(completion, last_time + abs(target - actual) / speed)
    duration = max(motion_duration, completion)

    def sampler(t: np.ndarray, order: int) -> np.ndarray:
        if order not in (0, 1, 2):
            raise ValueError(f"不支持的 path order：{order}")

        scalar = t.ndim == 0
        if motion_path is None:
            motion = np.zeros((1 if scalar else len(t), 3), dtype=float)
            if order == 0:
                motion[:] = motion_waypoints[-1]
        else:
            motion_t = np.clip(t, 0.0, motion_duration)
            motion = np.asarray(motion_path(motion_t, order=order), dtype=float)
            if order in (1, 2):
                stopped = np.asarray(t > motion_duration, dtype=bool)
                if stopped.ndim == 0:
                    if bool(stopped):
                        motion = np.zeros_like(motion)
                else:
                    motion[stopped] = 0.0

        if scalar:
            motion_2d = motion.reshape(1, 3)
            t_1d = t.reshape(1)
        else:
            motion_2d = motion
            t_1d = t

        servo = np.zeros((len(t_1d), 2), dtype=float)
        if order == 0:
            issue_times = waypoint_times[:-1]
            indices = np.searchsorted(issue_times, t_1d, side="right")
            indices = np.clip(indices, 0, len(servo_waypoints) - 1)
            servo = servo_waypoints[indices]

        result = np.hstack((motion_2d, servo))
        return result[0] if scalar else result

    return PlannedPath(sampler, duration)


def _move_towards(current: float, target: float, speed: float, dt: float) -> float:
    if speed <= 0.0:
        raise TrajectoryError("舵机速度上限必须大于 0。")
    if dt <= 0.0:
        return current
    delta = target - current
    max_step = speed * dt
    if abs(delta) <= max_step:
        return target
    return current + max_step if delta > 0.0 else current - max_step
