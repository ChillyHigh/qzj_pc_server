"""底盘碰撞检测与 3D A* 避障路径规划。

碰撞模型：以驱动轮中心为原点，前方 +CHASSIS_HALF_X_FRONT，后方 -CHASSIS_HALF_X_REAR，
左右 ±CHASSIS_HALF_Y 的矩形。A* 在 (x, y, yaw) 三维空间中搜索，分辨率 1cm / 10°。

只依赖 plan.setting 和 chassis.config，不引用 planner 包。
"""

from __future__ import annotations

import heapq
import math

from plan.setting import (
    CHASSIS_HALF_X_FRONT,
    CHASSIS_HALF_X_REAR,
    CHASSIS_HALF_Y,
    FIELD_X_MAX,
    FIELD_X_MIN,
    FIELD_Y_MAX,
    FIELD_Y_MIN,
    FUNNEL_SIDE_EXTENSION_Y,
    OBSTACLE_CENTERS,
    OBSTACLE_RADIUS,
    TARGET_RECTS,
)

from . import config

# ---- A* 搜索参数 -------------------------------------------------------------

_XY_RES = 0.01  # 1cm 分辨率
_YAW_RES = math.radians(10.0)  # 10° 一档
_YAW_BINS = 36

# 速度上限（代价函数用）
_V_MAX = min(config.V_LIMIT[0], config.V_LIMIT[1])
_W_MAX = config.V_LIMIT[2]

_MAX_EXPANSIONS = 300000
_LOS_YAW_STEP = math.radians(2.0)


# =============================================================================
# 几何工具
# =============================================================================


def _rotate(point: tuple[float, float], yaw: float) -> tuple[float, float]:
    """绕原点旋转 yaw 弧度。"""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * point[0] - s * point[1], s * point[0] + c * point[1])


def _oriented_chassis_corners(
    drive_x: float, drive_y: float, drive_yaw: float, half_y: float = CHASSIS_HALF_Y,
) -> tuple[tuple[float, float], ...]:
    """底盘四角世界坐标（不对称包络，以驱动轮中心为参考点）。

    底盘局部坐标系：+x 前方 215mm，-x 后方 195mm，±y 左右由 half_y 指定。
    """
    # 四个角的局部坐标：(front/rear, left/right)
    local_corners = (
        (CHASSIS_HALF_X_FRONT, half_y),    # 前左
        (CHASSIS_HALF_X_FRONT, -half_y),   # 前右
        (-CHASSIS_HALF_X_REAR, -half_y),   # 后右
        (-CHASSIS_HALF_X_REAR, half_y),    # 后左
    )
    corners = []
    for lx, ly in local_corners:
        offset = _rotate((lx, ly), drive_yaw)
        corners.append((drive_x + offset[0], drive_y + offset[1]))
    return tuple(corners)


def _rect_corners(
    rect: tuple[tuple[float, float], tuple[float, float]],
) -> tuple[tuple[float, float], ...]:
    """轴对齐矩形四角。"""
    center, half = rect
    return (
        (center[0] - half[0], center[1] - half[1]),
        (center[0] - half[0], center[1] + half[1]),
        (center[0] + half[0], center[1] + half[1]),
        (center[0] + half[0], center[1] - half[1]),
    )


def _polygon_axes(poly: tuple[tuple[float, float], ...]) -> list[tuple[float, float]]:
    """多边形边法向量（SAT 分离轴候选）。"""
    axes = []
    for idx, p in enumerate(poly):
        nxt = poly[(idx + 1) % len(poly)]
        edge = (nxt[0] - p[0], nxt[1] - p[1])
        length = math.hypot(*edge)
        if length > 1e-12:
            axes.append((-edge[1] / length, edge[0] / length))
    return axes


def _project_polygon(
    poly: tuple[tuple[float, float], ...], axis: tuple[float, float],
) -> tuple[float, float]:
    """多边形在轴上的投影区间 (min, max)。"""
    vals = [p[0] * axis[0] + p[1] * axis[1] for p in poly]
    return (min(vals), max(vals))


def _rects_overlap(
    chassis_corners: tuple[tuple[float, float], ...],
    rect: tuple[tuple[float, float], tuple[float, float]],
) -> bool:
    """SAT：底盘矩形与目标箱是否重叠。"""
    target = _rect_corners(rect)
    for axis in _polygon_axes(chassis_corners) + _polygon_axes(target):
        c0, c1 = _project_polygon(chassis_corners, axis)
        t0, t1 = _project_polygon(target, axis)
        if c1 <= t0 or t1 <= c0:
            return False
    return True


def _obstacle_collides(
    drive_x: float, drive_y: float, drive_yaw: float, half_y: float,
) -> bool:
    """底盘矩形与障碍柱的碰撞检测。

    将圆心转到底盘局部坐标系，用 AABB 到点距离判断。
    AABB 前方到 rear_x=+FRONT，后方到 front_x=-REAR，左右 ±half_y。
    """
    for ox, oy in OBSTACLE_CENTERS:
        local = _rotate((ox - drive_x, oy - drive_y), -drive_yaw)
        # 底盘局部 AABB：x ∈ [-REAR, +FRONT], y ∈ [-half_y, +half_y]
        dx = max(local[0] - CHASSIS_HALF_X_FRONT, -CHASSIS_HALF_X_REAR - local[0], 0.0)
        dy = max(abs(local[1]) - half_y, 0.0)
        if math.hypot(dx, dy) <= OBSTACLE_RADIUS:
            return True
    return False


def _is_chassis_inside_field(drive_x: float, drive_y: float, drive_yaw: float) -> bool:
    for x, y in _oriented_chassis_corners(drive_x, drive_y, drive_yaw):
        if x < FIELD_X_MIN or x > FIELD_X_MAX or y < FIELD_Y_MIN or y > FIELD_Y_MAX:
            return False
    return True


def is_drive_pose_colliding(
    drive_x: float, drive_y: float, drive_yaw: float,
    *,
    skip_boxes: bool = False,
) -> bool:
    """底盘在指定位姿是否碰撞。

    Args:
        skip_boxes: True 时跳过目标箱检查（端点使用，位姿由 planner 保证）。
    """
    if not _is_chassis_inside_field(drive_x, drive_y, drive_yaw):
        return True
    base_corners = _oriented_chassis_corners(drive_x, drive_y, drive_yaw)
    funnel_corners = _oriented_chassis_corners(
        drive_x,
        drive_y,
        drive_yaw,
        CHASSIS_HALF_Y + FUNNEL_SIDE_EXTENSION_Y,
    )

    if _obstacle_collides(
        drive_x,
        drive_y,
        drive_yaw,
        CHASSIS_HALF_Y + FUNNEL_SIDE_EXTENSION_Y,
    ):
        return True
    if not skip_boxes:
        for pos_id, rect in TARGET_RECTS.items():
            if _rects_overlap(base_corners, rect):
                return True
            if pos_id <= 3 and _rects_overlap(funnel_corners, rect):
                return True
    return False


# =============================================================================
# 3D A* (x, y, yaw)
# =============================================================================

def _xy_to_ix(x: float) -> int:
    return round((x - FIELD_X_MIN) / _XY_RES)


def _xy_to_iy(y: float) -> int:
    return round((y - FIELD_Y_MIN) / _XY_RES)


def _yaw_to_iaz(yaw: float) -> int:
    return round((yaw % (2.0 * math.pi)) / _YAW_RES) % _YAW_BINS


def _ix_to_x(ix: int) -> float:
    return FIELD_X_MIN + ix * _XY_RES


def _iy_to_y(iy: int) -> float:
    return FIELD_Y_MIN + iy * _XY_RES


def _iaz_to_yaw(iaz: int) -> float:
    return iaz * _YAW_RES % (2.0 * math.pi)


_IX_COUNT = _xy_to_ix(FIELD_X_MAX) + 1
_IY_COUNT = _xy_to_iy(FIELD_Y_MAX) + 1


def _encode(ix: int, iy: int, iaz: int) -> int:
    return (ix * _IY_COUNT + iy) * _YAW_BINS + iaz


def _decode(key: int) -> tuple[int, int, int]:
    iaz = key % _YAW_BINS
    rest = key // _YAW_BINS
    iy = rest % _IY_COUNT
    ix = rest // _IY_COUNT
    return (ix, iy, iaz)


def _is_within_field(ix: int, iy: int) -> bool:
    return 0 <= ix < _IX_COUNT and 0 <= iy < _IY_COUNT


def _heuristic(ix: int, iy: int, iaz: int, gix: int, giy: int, giaz: int) -> float:
    """可纳启发：欧氏距离/v_max + 角度差/ω_max。"""
    dx = _ix_to_x(gix) - _ix_to_x(ix)
    dy = _iy_to_y(giy) - _iy_to_y(iy)
    dist = math.hypot(dx, dy)
    d = abs(_iaz_to_yaw(giaz) - _iaz_to_yaw(iaz)) % (2.0 * math.pi)
    if d > math.pi:
        d = 2.0 * math.pi - d
    return dist / _V_MAX + d / _W_MAX


_MOVE_STEP = 1  # 1 格 = 0.01m
_YAW_STEP = 1   # 1 格 = 10°


def _expand_state(
    ix: int, iy: int, iaz: int,
) -> list[tuple[int, int, int, float]]:
    """展开动作原语：世界坐标系平移 + 原地旋转。

    全向轮底盘平移方向与 yaw 无关，用世界坐标避免无意义旋转：
    想往左走直接 -x，不需要先转 90° 再"前进"。
    yaw 只在碰撞检测和到达目标姿态时起作用。
    """
    move_cost = _XY_RES / _V_MAX
    rot_cost = _YAW_RES / _W_MAX
    return [
        # 世界坐标系平移（yaw 不变）
        (ix + _MOVE_STEP, iy, iaz, move_cost),           # +x
        (ix - _MOVE_STEP, iy, iaz, move_cost),           # -x
        (ix, iy + _MOVE_STEP, iaz, move_cost),           # +y
        (ix, iy - _MOVE_STEP, iaz, move_cost),           # -y
        # 旋转（xy 不变）
        (ix, iy, (iaz + _YAW_STEP) % _YAW_BINS, rot_cost),  # +10°
        (ix, iy, (iaz - _YAW_STEP) % _YAW_BINS, rot_cost),  # -10°
    ]


def _shortest_yaw_delta(from_yaw: float, to_yaw: float) -> float:
    """最短角增量，范围 [-π, π)。"""
    return (to_yaw - from_yaw + math.pi) % (2.0 * math.pi) - math.pi


def _line_of_sight(
    ix1: int, iy1: int, iaz1: int,
    ix2: int, iy2: int, iaz2: int,
    col_cache: dict[int, bool],
) -> bool:
    """Theta* 视线检查：两点间直线路径是否无碰撞。

    1cm 分辨率连续采样，碰撞结果通过 col_cache 缓存。
    """
    x1, y1, yaw1 = _ix_to_x(ix1), _iy_to_y(iy1), _iaz_to_yaw(iaz1)
    x2, y2, yaw2 = _ix_to_x(ix2), _iy_to_y(iy2), _iaz_to_yaw(iaz2)

    dyaw = _shortest_yaw_delta(yaw1, yaw2)
    dist = math.hypot(x2 - x1, y2 - y1)
    n_samples = max(2, int(math.ceil(dist / _XY_RES)), int(math.ceil(abs(dyaw) / _LOS_YAW_STEP)))

    for t in range(1, n_samples):
        alpha = t / n_samples
        x = x1 + alpha * (x2 - x1)
        y = y1 + alpha * (y2 - y1)
        yaw = yaw1 + alpha * dyaw
        cix, ciy, ciaz = _xy_to_ix(x), _xy_to_iy(y), _yaw_to_iaz(yaw)
        c_key = _encode(cix, ciy, ciaz)
        if c_key not in col_cache:
            col_cache[c_key] = is_drive_pose_colliding(x, y, yaw)
        if col_cache[c_key]:
            return False
    return True


def _direct_cost(ix1: int, iy1: int, iaz1: int, ix2: int, iy2: int, iaz2: int) -> float:
    """Theta* 直接移动的时间代价（与启发函数一致）。"""
    dx = _ix_to_x(ix2) - _ix_to_x(ix1)
    dy = _iy_to_y(iy2) - _iy_to_y(iy1)
    dist = math.hypot(dx, dy)
    dyaw = abs(_shortest_yaw_delta(_iaz_to_yaw(iaz1), _iaz_to_yaw(iaz2)))
    return dist / _V_MAX + dyaw / _W_MAX


def plan_avoidance_path(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    """Theta*（any-angle A*）搜索无碰撞时间最优路径。

    相比标准 A*，Theta* 在展开节点时额外检查从当前节点的父节点
    到邻居节点是否有视线（line-of-sight）。如有，则可以直接从祖父
    跳到邻居，绕过网格约束，产生真正的直线路径。
    """
    six, siy, siaz = _xy_to_ix(start[0]), _xy_to_iy(start[1]), _yaw_to_iaz(start[2])
    gix, giy, giaz = _xy_to_ix(end[0]), _xy_to_iy(end[1]), _yaw_to_iaz(end[2])

    # 端点只查障碍柱（不查箱，位姿由 planner 保证）
    if not _is_within_field(six, siy):
        raise ValueError(f"起点 {start} 超出场地")
    if is_drive_pose_colliding(start[0], start[1], start[2], skip_boxes=True):
        raise ValueError(f"起点 {start} 与障碍柱碰撞")
    if not _is_within_field(gix, giy):
        raise ValueError(f"终点 {end} 超出场地")
    if is_drive_pose_colliding(end[0], end[1], end[2], skip_boxes=True):
        raise ValueError(f"终点 {end} 与障碍柱碰撞")

    s_key = _encode(six, siy, siaz)
    g_key = _encode(gix, giy, giaz)
    g_score: dict[int, float] = {s_key: 0.0}
    parent: dict[int, int] = {}
    open_set: list[tuple[float, int]] = []
    heapq.heappush(open_set, (_heuristic(six, siy, siaz, gix, giy, giaz), s_key))
    closed: set[int] = set()
    col_cache: dict[int, bool] = {}  # 网格碰撞缓存，避免重复 SAT 计算

    expansions = 0
    while open_set and expansions < _MAX_EXPANSIONS:
        _, cur = heapq.heappop(open_set)
        if cur in closed:
            continue
        if cur == g_key:
            return _reconstruct_path(parent, cur, start, end)
        closed.add(cur)
        expansions += 1
        cix, ciy, ciaz = _decode(cur)
        cur_parent = parent.get(cur)

        for nix, niy, niaz, step_cost in _expand_state(cix, ciy, ciaz):
            if not _is_within_field(nix, niy):
                continue
            n_key = _encode(nix, niy, niaz)
            if n_key in closed:
                continue
            # 路径中间点全量碰撞检查（箱 + 柱），走缓存
            if n_key not in col_cache:
                col_cache[n_key] = is_drive_pose_colliding(
                    _ix_to_x(nix), _iy_to_y(niy), _iaz_to_yaw(niaz),
                )
            if col_cache[n_key]:
                continue

            # ── Theta*: 尝试从祖父直接跳到邻居 ──
            if cur_parent is not None:
                pix, piy, piaz = _decode(cur_parent)
                if _line_of_sight(pix, piy, piaz, nix, niy, niaz, col_cache):
                    tg = g_score[cur_parent] + _direct_cost(pix, piy, piaz, nix, niy, niaz)
                    if tg < g_score.get(n_key, float("inf")):
                        g_score[n_key] = tg
                        parent[n_key] = cur_parent
                        f = tg + _heuristic(nix, niy, niaz, gix, giy, giaz)
                        heapq.heappush(open_set, (f, n_key))
                    continue  # LoS 存在则跳过标准网格扩展

            # ── 标准网格：从当前节点走一步 ──
            tg = g_score[cur] + step_cost
            if tg < g_score.get(n_key, float("inf")):
                g_score[n_key] = tg
                parent[n_key] = cur
                f = tg + _heuristic(nix, niy, niaz, gix, giy, giaz)
                heapq.heappush(open_set, (f, n_key))

    if expansions >= _MAX_EXPANSIONS:
        raise ValueError(f"Theta* 超过最大展开 {_MAX_EXPANSIONS}")
    raise ValueError(f"Theta* 无路径 ({start[0]:.2f},{start[1]:.2f})→({end[0]:.2f},{end[1]:.2f})")


def _unwrap_path_yaw(
    path: list[tuple[float, float, float]],
) -> list[tuple[float, float, float]]:
    """对路径的 yaw 维度解缠绕，使相邻点 yaw 差 ≤ π。

    yaw 可累积到任意值（如 1.5π→2.5π），不做范围约束，
    只保证相邻点之间始终走最短角路径。
    """
    result = [path[0]]
    accum = path[0][2]
    for i in range(1, len(path)):
        prev = accum
        raw = path[i][2]
        d = (raw - prev + math.pi) % (2.0 * math.pi) - math.pi
        accum = prev + d
        result.append((path[i][0], path[i][1], accum))
    return result


def _reconstruct_path(
    parent: dict[int, int], goal_key: int,
    start: tuple[float, float, float], end: tuple[float, float, float],
) -> list[tuple[float, float, float]]:
    """回溯 + 共线合并 + yaw 解缠绕。"""
    raw: list[tuple[float, float, float]] = []
    key = goal_key
    while key in parent:
        ix, iy, iaz = _decode(key)
        raw.append((_ix_to_x(ix), _iy_to_y(iy), _iaz_to_yaw(iaz)))
        key = parent[key]
    raw.reverse()
    if not raw:
        return [start, end]

    path = [start, *raw]
    path[-1] = end
    # yaw 解缠绕：使路径 yaw 连续，解决 config yaw（可能 [-π,π]）
    # 与 A* yaw（[0,2π)）混用导致的 2π 跳变
    path = _unwrap_path_yaw(path)

    if len(path) <= 2:
        return path

    # 共线合并（yaw 已解缠绕，直接比较差值即可）
    merged = [path[0]]
    for i in range(1, len(path) - 1):
        p, c, n = merged[-1], path[i], path[i + 1]
        v1 = (c[0] - p[0], c[1] - p[1])
        v2 = (n[0] - c[0], n[1] - c[1])
        cross = v1[0] * v2[1] - v1[1] * v2[0]
        if abs(cross) < 1e-6 and abs(c[2] - p[2]) < 1e-6 and abs(n[2] - c[2]) < 1e-6:
            continue
        merged.append(c)
    merged.append(path[-1])
    return merged


# =============================================================================
# 路径校验
# =============================================================================

_VALIDATE_YAW_STEP = math.radians(2.0)


def validate_chassis_path(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
) -> None:
    """沿直线路径采样，检查是否碰撞（端点位姿由 planner 保证，跳过）。"""
    dist = math.hypot(end[0] - start[0], end[1] - start[1])
    dyaw = (end[2] - start[2] + math.pi) % (2.0 * math.pi) - math.pi
    sample_count = max(
        2,
        int(math.ceil(dist / _XY_RES)),
        int(math.ceil(abs(dyaw) / _VALIDATE_YAW_STEP)),
    )
    for i in range(1, sample_count):  # 跳过 t=0 和 t=1
        t = i / sample_count
        x = start[0] + t * (end[0] - start[0])
        y = start[1] + t * (end[1] - start[1])
        yaw = start[2] + t * dyaw
        if is_drive_pose_colliding(x, y, yaw):
            raise ValueError(
                f"chassis 路径 t={t:.2f} ({x:.3f},{y:.3f},{math.degrees(yaw):.0f}°) 碰撞"
            )
