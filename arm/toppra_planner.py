from __future__ import annotations

import numpy as np
from toppra.interpolator import AbstractGeometricPath

from trajectory import ToppraPlanner, TrajectoryError, Waypoint

from . import config


class ArmPathError(ValueError):
    """arm path 生成失败。"""


class ArmServoPath(AbstractGeometricPath):
    """5 维 arm path，其中舵机维度只输出目标角。"""

    def __init__(
        self,
        motion_path: AbstractGeometricPath | None,
        motion_waypoints: np.ndarray,
        servo_waypoints: np.ndarray,
        servo_v_limit: np.ndarray,
    ) -> None:
        self.motion_path = motion_path
        self.motion_waypoints = motion_waypoints
        self.servo_waypoints = servo_waypoints
        self.servo_v_limit = servo_v_limit
        self.motion_duration = 0.0 if motion_path is None else float(motion_path.duration)
        self.duration = self.motion_duration

        if motion_path is None:
            self.waypoint_times = np.zeros(len(motion_waypoints), dtype=float)
        else:
            t_grid = np.linspace(0.0, self.motion_duration, max(2, len(motion_waypoints) * 20))
            motion_samples = np.asarray(motion_path(t_grid, order=0), dtype=float)
            waypoint_times = []
            for waypoint_q in motion_waypoints:
                nearest_idx = int(np.argmin(np.linalg.norm(motion_samples - waypoint_q, axis=1)))
                waypoint_times.append(float(t_grid[nearest_idx]))
            self.waypoint_times = np.maximum.accumulate(np.asarray(waypoint_times, dtype=float))
            self.waypoint_times[0] = 0.0
            self.waypoint_times[-1] = self.motion_duration

        self.duration = max(self.motion_duration, self._servo_completion_time())

    @property
    def dof(self) -> int:
        return 5

    @property
    def path_interval(self):
        return np.asarray([0.0, self.duration], dtype=float)

    @property
    def waypoints(self):
        return None

    def __call__(self, path_positions, order: int = 0) -> np.ndarray:
        t = np.asarray(path_positions, dtype=float)
        scalar = t.ndim == 0

        if self.motion_path is None:
            motion = np.zeros((1 if scalar else len(t), 3), dtype=float)
            if order == 0:
                motion[:] = self.motion_waypoints[-1]
        else:
            motion_t = np.clip(t, 0.0, self.motion_duration)
            motion = np.asarray(self.motion_path(motion_t, order=order), dtype=float)
            if order in (1, 2):
                motion = np.asarray(motion, dtype=float)
                stopped = np.asarray(t > self.motion_duration, dtype=bool)
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
            issue_times = self.waypoint_times[:-1]
            indices = np.searchsorted(issue_times, t_1d, side="right")
            indices = np.clip(indices, 0, len(self.servo_waypoints) - 1)
            servo = self.servo_waypoints[indices]
        elif order not in (1, 2):
            raise ValueError(f"不支持的 path order：{order}")

        result = np.hstack((motion_2d, servo))
        return result[0] if scalar else result

    def _servo_completion_time(self) -> float:
        if len(self.servo_waypoints) < 2:
            return 0.0

        completion = 0.0
        for dim in range(2):
            actual = float(self.servo_waypoints[0, dim])
            target = actual
            last_time = 0.0
            speed = float(self.servo_v_limit[dim])
            for idx in range(1, len(self.servo_waypoints)):
                issue_time = float(self.waypoint_times[idx - 1])
                actual = _move_towards(actual, target, speed, issue_time - last_time)
                target = float(self.servo_waypoints[idx, dim])
                last_time = issue_time
            completion = max(completion, last_time + abs(target - actual) / speed)
        return completion


class ArmToppraPlanner(ToppraPlanner):
    """Arm TOPPRA planner。

    h/q1/q2 参与 TOPPRA 速度和加速度约束；gripper_yaw/gripper_opening
    是舵机目标角，只按同一时间轴输出目标值，不生成速度前馈。
    """

    def plan(
        self,
        waypoints: list[Waypoint],
        *,
        max_step: float = 0.05,
    ) -> AbstractGeometricPath:
        if len(waypoints) < 2:
            raise TrajectoryError("Arm TOPPRA 至少需要 2 个航点。")
        self._check_arm_waypoints(waypoints)

        motion_waypoints = [
            Waypoint(
                q=tuple(float(value) for value in waypoint.q[:3]),
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
            return ArmServoPath(None, motion_qs, servo_qs, servo_v_limit)
        motion_path = super().plan(motion_waypoints, max_step=max_step)
        return ArmServoPath(motion_path, motion_qs, servo_qs, servo_v_limit)

    def _check_arm_waypoints(self, waypoints: list[Waypoint]) -> None:
        q_min = np.asarray(config.Q_MIN_LIMIT, dtype=float)
        q_max = np.asarray(config.Q_MAX_LIMIT, dtype=float)
        for idx, waypoint in enumerate(waypoints):
            q = np.asarray(waypoint.q, dtype=float)
            if q.shape != (5,):
                raise TrajectoryError(f"arm 航点[{idx}] 维度必须为 5。")
            if not np.all(np.isfinite(q)):
                raise TrajectoryError(f"arm 航点[{idx}] 包含 NaN 或无穷大。")
            if np.any(q < q_min):
                raise TrajectoryError(f"arm 航点[{idx}] 小于 q_min。")
            if np.any(q > q_max):
                raise TrajectoryError(f"arm 航点[{idx}] 大于 q_max。")


def plan_joint_waypoints(waypoints: list[Waypoint]) -> AbstractGeometricPath:
    """对已经 IK 后的 arm joint-space 航点做 TOPPRA。"""

    planner = ArmToppraPlanner(
        config.MOTION_V_LIMIT,
        config.MOTION_A_LIMIT,
    )
    return planner.plan(waypoints)


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
