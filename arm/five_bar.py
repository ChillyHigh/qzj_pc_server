from __future__ import annotations

import math
from itertools import product

from . import config
from .types import ArmKinematicsError, FiveBarParams


def _circle_intersections(
    c1x: float,
    c1y: float,
    r1: float,
    c2x: float,
    c2y: float,
    r2: float,
) -> tuple[tuple[float, float], tuple[float, float]]:
    dx = c2x - c1x
    dy = c2y - c1y
    d = math.hypot(dx, dy)
    if d <= 0.0:
        raise ArmKinematicsError("两个圆心重合，无法求交点。")
    if d > r1 + r2:
        raise ArmKinematicsError("两圆相离，无法闭合五连杆。")
    if d < abs(r1 - r2):
        raise ArmKinematicsError("一圆包含另一圆，无法闭合五连杆。")

    a = (r1 * r1 - r2 * r2 + d * d) / (2.0 * d)
    h_sq = r1 * r1 - a * a
    if h_sq < -1e-9:
        raise ArmKinematicsError("圆交计算出现负值。")
    h = math.sqrt(max(h_sq, 0.0))

    px = c1x + a * dx / d
    py = c1y + a * dy / d
    rx = -dy * h / d
    ry = dx * h / d
    return (px + rx, py + ry), (px - rx, py - ry)


def _upper_elbow_from_q(motor: tuple[float, float], active_link: float, q: float) -> tuple[float, float]:
    return (
        motor[0] + active_link * math.sin(q),
        motor[1] - active_link * math.cos(q),
    )


def _lower_elbow_from_q(motor: tuple[float, float], active_link: float, q: float) -> tuple[float, float]:
    return (
        motor[0] - active_link * math.sin(q),
        motor[1] + active_link * math.cos(q),
    )


def _upper_elbow_q(elbow_x: float, elbow_y: float, motor_x: float, motor_y: float, params: FiveBarParams) -> float | None:
    q = _normalize_q(math.atan2(elbow_x - motor_x, motor_y - elbow_y))
    if params.q_min - 1e-9 <= q <= params.q_max + 1e-9:
        return min(max(q, params.q_min), params.q_max)
    return None


def _lower_elbow_q(elbow_x: float, elbow_y: float, motor_x: float, motor_y: float, params: FiveBarParams) -> float | None:
    q = _normalize_q(math.atan2(motor_x - elbow_x, elbow_y - motor_y))
    if params.q_min - 1e-9 <= q <= params.q_max + 1e-9:
        return min(max(q, params.q_min), params.q_max)
    return None


def _normalize_q(q: float) -> float:
    normalized = q % (2.0 * math.pi)
    if math.isclose(normalized, 2.0 * math.pi, abs_tol=1e-12):
        return 0.0
    return normalized


def _wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_2pi(angle: float) -> float:
    return angle % (2.0 * math.pi)


def _angle_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def _line_y_at_x(line_a: tuple[float, float], line_b: tuple[float, float], x: float) -> float | None:
    dx = line_b[0] - line_a[0]
    if abs(dx) <= 1e-9:
        return None
    return line_a[1] + (line_b[1] - line_a[1]) * (x - line_a[0]) / dx


def _elbow_outward(
    motor: tuple[float, float],
    elbow: tuple[float, float],
    endpoint: tuple[float, float],
) -> bool:
    line_y = _line_y_at_x(motor, endpoint, elbow[0])
    if line_y is None:
        if abs(elbow[0] - motor[0]) > 1e-9:
            return False
        if motor[1] > 0.0:
            return elbow[1] >= motor[1] - 1e-9
        return elbow[1] <= motor[1] + 1e-9
    if motor[1] > 0.0:
        return elbow[1] >= line_y - 1e-9
    return elbow[1] <= line_y + 1e-9


def _elbows_outward(
    upper_motor: tuple[float, float],
    lower_motor: tuple[float, float],
    upper_elbow: tuple[float, float],
    lower_elbow: tuple[float, float],
    endpoint: tuple[float, float],
) -> bool:
    return (
        _elbow_outward(upper_motor, upper_elbow, endpoint)
        and _elbow_outward(lower_motor, lower_elbow, endpoint)
    )


def _matches_outward_branch(q1: float, q2: float, endpoint: tuple[float, float]) -> bool:
    q_delta = q2 - q1
    if abs(q_delta) > 1e-9 and endpoint[0] * q_delta < -1e-9:
        return False
    y_delta = (q1 + q2 - 2.0 * math.pi) * q_delta
    if abs(y_delta) > 1e-9 and endpoint[1] * y_delta < -1e-9:
        return False
    return True


def _check_q(params: FiveBarParams, q: float, name: str) -> None:
    if not math.isfinite(q):
        raise ArmKinematicsError(f"{name} 不是有限值：{q}")
    if not (params.q_min <= q <= params.q_max):
        raise ArmKinematicsError(
            f"{name} 超出范围 [{params.q_min:.4f}, {params.q_max:.4f}]：{q:.4f}"
        )


def _check_ik_margin(
    x: float,
    y: float,
    params: FiveBarParams,
    upper_motor: tuple[float, float],
    lower_motor: tuple[float, float],
) -> None:
    outer = params.passive_link + params.active_link
    margin = config.IK_DISTANCE_MARGIN
    max_dist = outer - margin
    if max_dist <= 0.0:
        raise ArmKinematicsError(
            f"IK_DISTANCE_MARGIN 过大：outer={outer:.4f}, margin={margin:.4f}"
        )
    for name, motor in (("upper", upper_motor), ("lower", lower_motor)):
        dist = math.hypot(x - motor[0], y - motor[1])
        if dist > max_dist:
            raise ArmKinematicsError(
                f"末端点距离 {name} 电机过远：dist={dist:.4f}, "
                f"max={max_dist:.4f}, x={x:.4f}, y={y:.4f}"
            )


def _inverse_options(
    x: float,
    y: float,
    params: FiveBarParams,
    upper_motor: tuple[float, float],
    lower_motor: tuple[float, float],
) -> tuple[tuple[float, float, tuple[float, float], tuple[float, float]], ...]:
    _check_ik_margin(x, y, params, upper_motor, lower_motor)
    upper_elbows = _circle_intersections(
        upper_motor[0],
        upper_motor[1],
        params.active_link,
        x,
        y,
        params.passive_link,
    )
    lower_elbows = _circle_intersections(
        lower_motor[0],
        lower_motor[1],
        params.active_link,
        x,
        y,
        params.passive_link,
    )

    solutions: list[tuple[float, float, tuple[float, float], tuple[float, float]]] = []
    for upper_elbow, lower_elbow in product(upper_elbows, lower_elbows):
        endpoint = (x, y)
        if not _elbows_outward(upper_motor, lower_motor, upper_elbow, lower_elbow, endpoint):
            continue
        q1 = _upper_elbow_q(
            upper_elbow[0],
            upper_elbow[1],
            upper_motor[0],
            upper_motor[1],
            params,
        )
        q2 = _lower_elbow_q(
            lower_elbow[0],
            lower_elbow[1],
            lower_motor[0],
            lower_motor[1],
            params,
        )
        if q1 is not None and q2 is not None:
            solutions.append((q1, q2, upper_elbow, lower_elbow))

    if not solutions:
        raise ArmKinematicsError(f"末端点不可达：x={x:.4f}, y={y:.4f}")
    return tuple(solutions)


def _select_endpoint(
    q1: float,
    q2: float,
    endpoints: tuple[tuple[float, float], tuple[float, float]],
    upper_motor: tuple[float, float],
    lower_motor: tuple[float, float],
    upper_elbow: tuple[float, float],
    lower_elbow: tuple[float, float],
) -> tuple[float, float]:
    compatible = [
        endpoint for endpoint in endpoints
        if _elbows_outward(upper_motor, lower_motor, upper_elbow, lower_elbow, endpoint)
        and _matches_outward_branch(q1, q2, endpoint)
    ]
    if not compatible:
        raise ArmKinematicsError("FK 未找到满足肘朝外约束的闭链分支。")
    if len(compatible) > 1:
        first = compatible[0]
        if all(math.hypot(point[0] - first[0], point[1] - first[1]) <= 1e-9 for point in compatible[1:]):
            return first
        raise ArmKinematicsError("FK 肘朝外约束没有唯一闭链分支。")
    return compatible[0]


class FiveBarKinematics:
    """五连杆平面运动学。

    对外契约只有：
    - `ik(x, y, yaw) -> (q1, q2, gripper_yaw)`
    - `fk(q1, q2, gripper_yaw) -> (x, y, yaw)`

    `x/y/yaw` 使用 arm 局部、与底盘对齐的坐标系；`yaw` 相对底盘。
    执行帧中的 `gripper_yaw=0` 表示夹爪相对 y<0 下侧从动杆反向。
    """

    def __init__(self, params: FiveBarParams | None = None) -> None:
        self.params = params or FiveBarParams()
        self.upper_motor = (self.params.motor_x, self.params.motor_y_offset)
        self.lower_motor = (self.params.motor_x, -self.params.motor_y_offset)

    def ik(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        options = _inverse_options(x, y, self.params, self.upper_motor, self.lower_motor)
        q1, q2, _, lower_elbow = min(
            options,
            key=lambda option: (
                _angle_distance(option[0], math.radians(210.0))
                + _angle_distance(option[1], math.radians(210.0))
            ),
        )
        lower_passive_yaw = math.atan2(y - lower_elbow[1], x - lower_elbow[0])
        return q1, q2, _wrap_2pi(yaw - lower_passive_yaw - math.pi)

    def fk(self, q1: float, q2: float, gripper_yaw: float) -> tuple[float, float, float]:
        _check_q(self.params, q1, "q1")
        _check_q(self.params, q2, "q2")
        upper_elbow = _upper_elbow_from_q(self.upper_motor, self.params.active_link, q1)
        lower_elbow = _lower_elbow_from_q(self.lower_motor, self.params.active_link, q2)
        endpoints = _circle_intersections(
            upper_elbow[0],
            upper_elbow[1],
            self.params.passive_link,
            lower_elbow[0],
            lower_elbow[1],
            self.params.passive_link,
        )
        endpoint = _select_endpoint(
            q1,
            q2,
            endpoints,
            self.upper_motor,
            self.lower_motor,
            upper_elbow,
            lower_elbow,
        )

        lower_passive_yaw = math.atan2(endpoint[1] - lower_elbow[1], endpoint[0] - lower_elbow[0])
        return endpoint[0], endpoint[1], _wrap_pi(lower_passive_yaw + gripper_yaw + math.pi)
