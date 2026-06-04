from __future__ import annotations

from typing import Any

import numpy as np
from scipy.interpolate import BSpline

from .densify import EPSILON
from .types import TrajectoryError


def find_corner_windows(
    junctions: list[dict[str, Any]],
    points: np.ndarray,
    min_turn_deg: float,
    window_ratio: float,
) -> list[dict[str, Any]]:
    """根据转角和窗口比例找出需要 B 样条倒角的点段。"""

    windows: list[dict[str, Any]] = []
    for junction in junctions:
        if junction["turn_deg"] is None or junction["turn_deg"] < min_turn_deg:
            continue

        junction_idx = junction["index"]
        left_len = _polyline_arc_length(points, 0, junction_idx + 1)
        right_len = _polyline_arc_length(points, junction_idx, len(points))
        cut = min(window_ratio * left_len, window_ratio * right_len)
        if cut <= EPSILON:
            continue

        start = _find_index_at_distance(points, junction_idx, cut, backward=True)
        end = _find_index_at_distance(points, junction_idx, cut, backward=False)
        if end - start >= 3:
            windows.append(
                {
                    "junction_idx": junction_idx,
                    "start": start,
                    "end": end,
                    "turn_deg": junction["turn_deg"],
                }
            )
    return windows


def smooth_corners(points: np.ndarray, windows: list[dict[str, Any]], target_spacing: float) -> np.ndarray:
    """对指定窗口做 B 样条倒角。

    如果某个窗口拟合失败，会跳过该窗口；这不是兜底执行，而是保持原折线轨迹。
    """

    result = points.copy()
    for window in sorted(windows, key=lambda item: item["start"], reverse=True):
        start = window["start"]
        end = window["end"]
        if end - start + 1 < 4:
            continue
        sparse = _sparsify_window(result, start, end, target_spacing)
        if len(sparse) > 6:
            junction_q = result[window["junction_idx"]]
            dists = np.linalg.norm(sparse - junction_q, axis=1)
            dists[0] = float("inf")
            dists[-1] = float("inf")
            sparse = np.delete(sparse, int(np.argmin(dists)), axis=0)
        try:
            result[start : end + 1] = _sample_bspline(_fit_bspline(sparse), end - start + 1)
        except (TrajectoryError, ValueError, TypeError):
            continue
    return result


def _polyline_arc_length(points: np.ndarray, start: int, end: int) -> float:
    if end - start < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points[start:end], axis=0), axis=1)))


def _find_index_at_distance(points: np.ndarray, from_idx: int, target_dist: float, backward: bool) -> int:
    if target_dist <= 0.0:
        return from_idx

    accumulated = 0.0
    if backward:
        idx = from_idx
        while idx > 0 and accumulated < target_dist:
            step = float(np.linalg.norm(points[idx] - points[idx - 1]))
            if accumulated + step >= target_dist:
                return idx
            accumulated += step
            idx -= 1
        return max(0, idx)

    idx = from_idx
    while idx < len(points) - 1 and accumulated < target_dist:
        step = float(np.linalg.norm(points[idx + 1] - points[idx]))
        if accumulated + step >= target_dist:
            return idx
        accumulated += step
        idx += 1
    return min(len(points) - 1, idx)


def _resample_uniform(points: np.ndarray, num_points: int) -> np.ndarray:
    if len(points) < 2:
        return points.copy()
    seg_lens = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cum_len = np.zeros(len(points), dtype=float)
    cum_len[1:] = np.cumsum(seg_lens)
    total_len = float(cum_len[-1])
    if total_len <= EPSILON:
        return np.tile(points[0], (num_points, 1))

    target_ss = np.linspace(0.0, total_len, num_points)
    result = []
    for target_s in target_ss:
        seg_idx = int(np.searchsorted(cum_len, target_s, side="right") - 1)
        seg_idx = max(0, min(seg_idx, len(seg_lens) - 1))
        seg_len = float(seg_lens[seg_idx])
        if seg_len <= EPSILON:
            result.append(points[seg_idx].copy())
            continue
        alpha = (target_s - cum_len[seg_idx]) / seg_len
        alpha = float(np.clip(alpha, 0.0, 1.0))
        result.append(points[seg_idx] * (1.0 - alpha) + points[seg_idx + 1] * alpha)
    return np.asarray(result, dtype=float)


def _sparsify_window(points: np.ndarray, start: int, end: int, target_spacing: float) -> np.ndarray:
    window = points[start : end + 1]
    arc_len = _polyline_arc_length(window, 0, len(window))
    min_spacing = arc_len / 6.0
    spacing = min(target_spacing, min_spacing) if min_spacing > 0.0 else target_spacing
    if arc_len <= spacing:
        return _resample_uniform(window, max(6, len(window)))
    raw_count = int(arc_len / spacing) + 1
    num_points = max(6, raw_count if raw_count % 2 == 0 else raw_count + 1)
    return _resample_uniform(window, num_points)


def _fit_bspline(control_points: np.ndarray) -> tuple[list[BSpline], np.ndarray]:
    degree = min(5, len(control_points) - 1)
    if degree < 1:
        raise TrajectoryError("B 样条至少需要 2 个控制点。")
    n_ctrl = len(control_points)
    n_interior = n_ctrl - degree - 1
    if n_interior > 0:
        interior = np.linspace(0.0, 1.0, n_interior + 2)[1:-1]
    else:
        interior = np.array([])
    knots = np.concatenate([np.zeros(degree + 1), interior, np.ones(degree + 1)])
    splines = [BSpline(knots, control_points[:, dim], degree) for dim in range(control_points.shape[1])]
    return splines, knots


def _sample_bspline(tck: tuple[list[BSpline], np.ndarray], num_samples: int) -> np.ndarray:
    splines, _ = tck
    t_grid = np.linspace(0.0, 1.0, num_samples)
    return np.column_stack([spline(t_grid) for spline in splines])
