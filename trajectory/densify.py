from __future__ import annotations

import math
from typing import Any

import numpy as np

from .types import TrajectoryError, Waypoint

EPSILON = 1e-9


def deduplicate_waypoints(waypoints: list[Waypoint], tol: float = 1e-6) -> list[Waypoint]:
    """移除连续重复航点。

    只移除相邻重复点；不跨段去重，避免破坏动作块边界。
    """

    if not waypoints:
        return []
    unique = [waypoints[0]]
    for waypoint in waypoints[1:]:
        if not np.allclose(waypoint.q, unique[-1].q, atol=tol):
            unique.append(waypoint)
    return unique


def sample_linear(q_start: np.ndarray, q_end: np.ndarray, max_step: float) -> np.ndarray:
    """按最大坐标步长做线性插值。"""

    delta = q_end - q_start
    max_delta = float(np.max(np.abs(delta)))
    segments = max(1, int(math.ceil(max_delta / max_step)))
    points = [q_start]
    for seg_idx in range(1, segments + 1):
        alpha = seg_idx / segments
        q = q_start * (1.0 - alpha) + q_end * alpha
        if np.linalg.norm(q - points[-1]) > EPSILON:
            points.append(q)
    return np.asarray(points, dtype=float)


def densify_segments(waypoints: list[Waypoint], max_step: float) -> tuple[np.ndarray, list[dict[str, Any]]]:
    """加密航点并返回需要倒角的连接点。

    倒角规则：
    - `source_kind` 相同且 `source_id` 相同：认为仍是同一个来源段，不倒角。
    - `source_kind` 变化：倒角。
    - `source_id` 变化：即使 kind 相同，也倒角。
    """

    if len(waypoints) < 2:
        return np.asarray([wp.q for wp in waypoints], dtype=float), []

    blocks = _split_blocks(waypoints)
    merged = blocks[0]["qs"].copy()
    junction_indices: list[dict[str, Any]] = []
    for idx in range(1, len(blocks)):
        left_block = blocks[idx - 1]
        right_block = blocks[idx]
        bridge = sample_linear(left_block["qs"][-1], right_block["qs"][0], max_step)

        if (
            left_block["source_kind"] != right_block["source_kind"]
            or left_block["source_id"] != right_block["source_id"]
        ):
            junction_indices.append(
                {
                    "index": len(merged) - 1,
                    "left_source_kind": left_block["source_kind"],
                    "right_source_kind": right_block["source_kind"],
                    "left_source_id": left_block["source_id"],
                    "right_source_id": right_block["source_id"],
                }
            )

        merged = _append_unique_rows(merged, bridge)
        merged = _append_unique_rows(merged, right_block["qs"])
    return merged, _compute_junction_turns(merged, junction_indices)


def compute_path_s(points: np.ndarray) -> np.ndarray:
    """按折线弧长生成 TOPPRA 的归一化路径参数 s。"""

    if len(points) <= 1:
        return np.zeros(len(points), dtype=float)
    dists = np.linalg.norm(np.diff(points, axis=0), axis=1)
    ss = np.zeros(len(points), dtype=float)
    ss[1:] = np.cumsum(dists)
    if ss[-1] <= 0.0:
        raise TrajectoryError("路径弧长为 0，无法进行 TOPPRA。")
    ss /= ss[-1]
    return ss


def _split_blocks(waypoints: list[Waypoint]) -> list[dict[str, Any]]:
    blocks: list[dict[str, Any]] = []
    i = 0
    while i < len(waypoints):
        waypoint = waypoints[i]
        if waypoint.source_kind == "segment" and waypoint.source_id is not None:
            source_id = waypoint.source_id
            block_waypoints = [waypoint]
            i += 1
            while i < len(waypoints):
                nxt = waypoints[i]
                if nxt.source_kind == "segment" and nxt.source_id == source_id:
                    block_waypoints.append(nxt)
                    i += 1
                else:
                    break
            blocks.append(
                {
                    "kind": "segment",
                    "source_kind": waypoint.source_kind,
                    "source_id": source_id,
                    "qs": np.asarray([wp.q for wp in block_waypoints], dtype=float),
                    "blend_single": False,
                }
            )
        else:
            blocks.append(
                {
                    "kind": "single",
                    "source_kind": waypoint.source_kind,
                    "source_id": waypoint.source_id,
                    "qs": np.asarray([waypoint.q], dtype=float),
                    "blend_single": bool(waypoint.blend_single),
                }
            )
            i += 1
    return blocks


def _append_unique_rows(base: np.ndarray, component: np.ndarray) -> np.ndarray:
    if len(component) == 0:
        return base
    shared = base[-1]
    append_points = component[1:] if np.linalg.norm(shared - component[0]) < EPSILON else component
    for q in append_points:
        if np.linalg.norm(q - base[-1]) > EPSILON:
            base = np.vstack([base, q])
    return base


def _compute_junction_turns(merged: np.ndarray, junction_indices: list[dict[str, Any]]) -> list[dict[str, Any]]:
    junctions: list[dict[str, Any]] = []
    last_index = -999
    for junction in junction_indices:
        junction_index = junction["index"]
        if junction_index <= last_index + 1:
            continue
        turn_deg = None
        if 0 < junction_index < len(merged) - 1:
            incoming = merged[junction_index] - merged[junction_index - 1]
            outgoing = merged[junction_index + 1] - merged[junction_index]
            incoming_norm = float(np.linalg.norm(incoming))
            outgoing_norm = float(np.linalg.norm(outgoing))
            if incoming_norm > EPSILON and outgoing_norm > EPSILON:
                turn_cos = float(
                    np.clip(
                        np.dot(incoming / incoming_norm, outgoing / outgoing_norm),
                        -1.0,
                        1.0,
                    )
                )
                turn_deg = float(np.degrees(np.arccos(turn_cos)))
        junctions.append(
            {
                "index": junction_index,
                "turn_deg": turn_deg,
                "left_source_kind": junction["left_source_kind"],
                "right_source_kind": junction["right_source_kind"],
                "left_source_id": junction["left_source_id"],
                "right_source_id": junction["right_source_id"],
            }
        )
        last_index = junction_index
    return junctions
