from __future__ import annotations

from typing import Any, Iterable

import numpy as np
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint

from .densify import compute_path_s, deduplicate_waypoints, densify_segments
from .smoothing import find_corner_windows, smooth_corners
from .types import ToppraResult, TrajectoryError, Waypoint

DEFAULT_SAMPLE_FREQ_HZ = 200.0
DEFAULT_MAX_STEP = 0.05
DEFAULT_MIN_TURN_DEG = 8.0
DEFAULT_WINDOW_RATIO = 0.6
DEFAULT_TARGET_SPACING = 0.08
TOPPRA_SOLVER = "seidel"


def densify_and_smooth(
    waypoints: list[Waypoint],
    *,
    max_step: float = DEFAULT_MAX_STEP,
    min_turn_deg: float = DEFAULT_MIN_TURN_DEG,
    window_ratio: float = DEFAULT_WINDOW_RATIO,
    target_spacing: float = DEFAULT_TARGET_SPACING,
) -> np.ndarray:
    """执行线性加密和连接处倒角，输出 TOPPRA 使用的几何路径点。"""

    waypoints = deduplicate_waypoints(waypoints)
    if len(waypoints) < 2:
        return np.asarray([wp.q for wp in waypoints], dtype=float)

    dense_qs, junctions = densify_segments(waypoints, max_step)
    windows = find_corner_windows(junctions, dense_qs, min_turn_deg, window_ratio)
    if not windows:
        return dense_qs
    return smooth_corners(dense_qs, windows, target_spacing)


class ToppraPlanner:
    """多维广义坐标的 TOPPRA 时间最优轨迹规划器。

    底盘可传入 q=(x,y,yaw)，arm 可传入
    q=(h,q1,q2,gripper_yaw,gripper_opening)。每一维允许单位不同，
    但对应的 vlim/alim 必须使用同一维度自己的物理单位。
    """

    def __init__(
        self,
        vlim: Iterable[float],
        alim: Iterable[float],
        *,
        q_min: Iterable[float] | None = None,
        q_max: Iterable[float] | None = None,
        solver: str = TOPPRA_SOLVER,
    ) -> None:
        """创建规划器。

        Args:
            vlim: 每个 q 维度的速度上限，单位与 q 维度一致。
            alim: 每个 q 维度的加速度上限，单位与 q 维度一致。
            q_min: 可选位置下限。
            q_max: 可选位置上限。
            solver: TOPPRA 求解器名。
        """

        self.vlim = np.asarray(tuple(vlim), dtype=float)
        self.alim = np.asarray(tuple(alim), dtype=float)
        self.q_min = None if q_min is None else np.asarray(tuple(q_min), dtype=float)
        self.q_max = None if q_max is None else np.asarray(tuple(q_max), dtype=float)
        self.solver = solver

    def plan(
        self,
        waypoints: list[Waypoint],
        *,
        sample_freq: float = DEFAULT_SAMPLE_FREQ_HZ,
        max_step: float = DEFAULT_MAX_STEP,
    ) -> ToppraResult:
        """生成带时间戳、速度和加速度的轨迹点。"""

        if len(waypoints) < 2:
            raise TrajectoryError("TOPPRA 至少需要 2 个航点。")
        self._check_waypoints(waypoints)

        waypoints = deduplicate_waypoints(waypoints)
        path_qs = densify_and_smooth(waypoints, max_step=max_step)
        path_s = compute_path_s(path_qs)
        path = ta.SplineInterpolator(path_s, path_qs)

        waypoint_qs = np.asarray([wp.q for wp in waypoints], dtype=float)
        waypoint_s = []
        for waypoint_q in waypoint_qs:
            nearest_idx = int(np.argmin(np.linalg.norm(path_qs - waypoint_q, axis=1)))
            waypoint_s.append(path_s[nearest_idx])
        waypoint_s_arr = np.asarray(waypoint_s, dtype=float)
        speed_scales = np.asarray([wp.speed_scale for wp in waypoints], dtype=float)

        def vlim_func(s_value: float) -> np.ndarray:
            scale = np.interp(s_value, waypoint_s_arr, speed_scales)
            scaled = self.vlim * scale
            return np.vstack((-scaled, scaled)).T

        pc_vel = constraint.JointVelocityConstraintVarying(vlim_func)
        pc_acc = constraint.JointAccelerationConstraint(np.vstack((-self.alim, self.alim)).T)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, solver_wrapper=self.solver, gridpoints=path_s)
        trajectory = instance.compute_trajectory(0.0, 0.0)
        if trajectory is None:
            raise TrajectoryError("TOPPRA 未能生成可行轨迹。")

        if sample_freq <= 0.0:
            raise TrajectoryError("sample_freq 必须大于 0。")
        dt = 1.0 / sample_freq
        num_samples = max(2, int(np.floor(trajectory.duration / dt)) + 1)
        t_grid = np.linspace(0.0, trajectory.duration, num_samples)
        q = trajectory(t_grid)
        dq = trajectory(t_grid, 1)
        ddq = trajectory(t_grid, 2)
        meta = _sample_meta(waypoints, waypoint_qs, t_grid, q)

        return ToppraResult(
            duration=float(trajectory.duration),
            t=t_grid,
            q=q,
            dq=dq,
            ddq=ddq,
            meta=meta,
            path_qs=path_qs,
            path_s=path_s,
        )

    def _check_waypoints(self, waypoints: list[Waypoint]) -> None:
        expected_dim = len(waypoints[0].q)
        if expected_dim != len(self.vlim) or expected_dim != len(self.alim):
            raise TrajectoryError("航点维度与速度/加速度限制维度不一致。")
        for idx, waypoint in enumerate(waypoints):
            q = np.asarray(waypoint.q, dtype=float)
            if len(q) != expected_dim:
                raise TrajectoryError(f"航点[{idx}] 维度不一致。")
            if not np.all(np.isfinite(q)):
                raise TrajectoryError(f"航点[{idx}] 包含 NaN 或无穷大。")
            if self.q_min is not None and np.any(q < self.q_min):
                raise TrajectoryError(f"航点[{idx}] 小于 q_min。")
            if self.q_max is not None and np.any(q > self.q_max):
                raise TrajectoryError(f"航点[{idx}] 大于 q_max。")


def _sample_meta(
    waypoints: list[Waypoint],
    waypoint_qs: np.ndarray,
    t_grid: np.ndarray,
    q: np.ndarray,
) -> list[dict[str, Any]]:
    waypoint_ts = []
    for waypoint_q in waypoint_qs:
        nearest_idx = int(np.argmin(np.linalg.norm(q - waypoint_q, axis=1)))
        waypoint_ts.append(t_grid[nearest_idx])
    waypoint_ts_arr = np.asarray(waypoint_ts, dtype=float)
    indices = np.searchsorted(waypoint_ts_arr, t_grid, side="right") - 1
    indices = np.clip(indices, 0, len(waypoints) - 1)
    return [dict(waypoints[int(index)].meta) for index in indices]
