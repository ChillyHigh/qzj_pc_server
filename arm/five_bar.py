from __future__ import annotations

import math
from itertools import product

import numpy as np

from .types import ArmKinematicsError, ArmSolution, FiveBarParams


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


def _elbow_q(
    motor_x: float,
    motor_y: float,
    elbow_x: float,
    elbow_y: float,
    params: FiveBarParams,
) -> float | None:
    q = math.atan2(motor_x - elbow_x, elbow_y - motor_y)
    if params.q_min - 1e-9 <= q <= params.q_max + 1e-9:
        return min(max(q, params.q_min), params.q_max)
    return None


def _angle_distance(a: float, b: float) -> float:
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


class FiveBarKinematics:
    """五连杆平面正逆运动学。

    约定：
    - 机器人局部坐标中，x 向右，y 向前/向上。
    - q=0 时主动杆指向局部 +y。
    - q 正方向与 yaw 一致，为逆时针，因此 q>0 时主动杆端点向 -x 转。

    五连杆对同一个末端点可能存在多组 q1/q2 解。轨迹规划应调用
    `ik_path` 并传入上一帧 q 作为参考，保证解在连续轨迹中不跳支。
    """

    def __init__(self, params: FiveBarParams | None = None) -> None:
        """创建五连杆模型。

        Args:
            params: 机构几何参数；为 None 时使用当前比赛车的默认尺寸。
        """

        self.params = params or FiveBarParams()
        self.upper_motor = (self.params.motor_x, self.params.motor_y_offset)
        self.lower_motor = (self.params.motor_x, -self.params.motor_y_offset)

    def forward_all(self, q1: float, q2: float) -> tuple[ArmSolution, ...]:
        """返回给定 q1/q2 下所有可能的末端闭链解。"""

        self._check_q(q1, "q1")
        self._check_q(q2, "q2")

        upper_elbow = (
            self.upper_motor[0] - self.params.active_link * math.sin(q1),
            self.upper_motor[1] + self.params.active_link * math.cos(q1),
        )
        lower_elbow = (
            self.lower_motor[0] - self.params.active_link * math.sin(q2),
            self.lower_motor[1] + self.params.active_link * math.cos(q2),
        )
        endpoints = _circle_intersections(
            upper_elbow[0],
            upper_elbow[1],
            self.params.passive_link,
            lower_elbow[0],
            lower_elbow[1],
            self.params.passive_link,
        )
        return tuple(
            ArmSolution(q1, q2, upper_elbow, lower_elbow, endpoint)
            for endpoint in endpoints
        )

    def forward(
        self,
        q1: float,
        q2: float,
        *,
        reference_endpoint: tuple[float, float] | None = None,
    ) -> ArmSolution:
        """返回一组正运动学解。

        Args:
            q1: 上侧主动杆角度，rad。
            q2: 下侧主动杆角度，rad。
            reference_endpoint: 如果给出，则选择离该末端点最近的闭链解。
        """

        options = self.forward_all(q1, q2)
        if reference_endpoint is not None:
            return min(
                options,
                key=lambda solution: (
                    (solution.endpoint[0] - reference_endpoint[0]) ** 2
                    + (solution.endpoint[1] - reference_endpoint[1]) ** 2
                ),
            )
        return max(options, key=lambda solution: solution.endpoint[0])

    def inverse_all(self, endpoint: tuple[float, float]) -> tuple[ArmSolution, ...]:
        """返回目标末端点的所有可行 IK 解。"""

        upper_elbows = _circle_intersections(
            self.upper_motor[0],
            self.upper_motor[1],
            self.params.active_link,
            endpoint[0],
            endpoint[1],
            self.params.passive_link,
        )
        lower_elbows = _circle_intersections(
            self.lower_motor[0],
            self.lower_motor[1],
            self.params.active_link,
            endpoint[0],
            endpoint[1],
            self.params.passive_link,
        )

        solutions: list[ArmSolution] = []
        for upper_elbow, lower_elbow in product(upper_elbows, lower_elbows):
            q1 = _elbow_q(self.upper_motor[0], self.upper_motor[1], upper_elbow[0], upper_elbow[1], self.params)
            q2 = _elbow_q(self.lower_motor[0], self.lower_motor[1], lower_elbow[0], lower_elbow[1], self.params)
            if q1 is None or q2 is None:
                continue
            solutions.append(ArmSolution(q1, q2, upper_elbow, lower_elbow, endpoint))

        if not solutions:
            raise ArmKinematicsError(f"末端点不可达：x={endpoint[0]:.4f}, y={endpoint[1]:.4f}")
        return tuple(solutions)

    def inverse(
        self,
        endpoint: tuple[float, float],
        *,
        reference_q: tuple[float, float] | None = None,
        preferred_q: tuple[float, float] = (math.radians(30.0), math.radians(150.0)),
    ) -> ArmSolution:
        """选择目标末端点的一组 IK 解。

        Args:
            endpoint: 目标末端点，单位 m。
            reference_q: 上一帧或当前帧 q；给出后优先选择角度变化最小的解。
            preferred_q: 无参考 q 时的默认偏好构型。
        """

        options = self.inverse_all(endpoint)
        if reference_q is not None:
            return min(
                options,
                key=lambda solution: (
                    _angle_distance(solution.q1, reference_q[0])
                    + _angle_distance(solution.q2, reference_q[1])
                ),
            )
        return min(
            options,
            key=lambda solution: (
                _angle_distance(solution.q1, preferred_q[0])
                + _angle_distance(solution.q2, preferred_q[1])
            ),
        )

    def ik_xy(
        self,
        x: float,
        y: float,
        *,
        reference_q: tuple[float, float] | None = None,
        preferred_q: tuple[float, float] = (math.radians(30.0), math.radians(150.0)),
    ) -> tuple[float, float]:
        """用标量坐标求 IK，返回 `(q1, q2)`。

        这是 arm 轨迹构造的热路径接口，不创建 `ArmSolution` 对象。
        """

        options = self._inverse_xy_options(x, y)
        if reference_q is not None:
            return min(
                options,
                key=lambda q: _angle_distance(q[0], reference_q[0]) + _angle_distance(q[1], reference_q[1]),
            )
        return min(
            options,
            key=lambda q: _angle_distance(q[0], preferred_q[0]) + _angle_distance(q[1], preferred_q[1]),
        )

    def ik_path(
        self,
        points_xy: np.ndarray,
        *,
        initial_q: tuple[float, float] | None = None,
    ) -> np.ndarray:
        """对 `Nx2` 末端点数组做连续 IK，返回 `Nx2` 的 q1/q2 数组。

        这是规划层生成 arm 航点的主接口。它复用上一点的 q 选解，
        避免五连杆从一个半平面跳到另一个半平面。
        """

        points = np.asarray(points_xy, dtype=float)
        if points.ndim != 2 or points.shape[1] != 2:
            raise ArmKinematicsError("points_xy 必须是形状为 Nx2 的数组。")
        result = np.empty((len(points), 2), dtype=float)
        reference_q = initial_q
        for idx, (x, y) in enumerate(points):
            q = self.ik_xy(float(x), float(y), reference_q=reference_q)
            result[idx, 0] = q[0]
            result[idx, 1] = q[1]
            reference_q = q
        return result

    def _check_q(self, q: float, name: str) -> None:
        if not math.isfinite(q):
            raise ArmKinematicsError(f"{name} 不是有限值：{q}")
        if not (self.params.q_min <= q <= self.params.q_max):
            raise ArmKinematicsError(
                f"{name} 超出范围 [{self.params.q_min:.4f}, {self.params.q_max:.4f}]：{q:.4f}"
            )

    def _inverse_xy_options(self, x: float, y: float) -> tuple[tuple[float, float], ...]:
        upper_elbows = _circle_intersections(
            self.upper_motor[0],
            self.upper_motor[1],
            self.params.active_link,
            x,
            y,
            self.params.passive_link,
        )
        lower_elbows = _circle_intersections(
            self.lower_motor[0],
            self.lower_motor[1],
            self.params.active_link,
            x,
            y,
            self.params.passive_link,
        )

        solutions: list[tuple[float, float]] = []
        for upper_elbow, lower_elbow in product(upper_elbows, lower_elbows):
            q1 = _elbow_q(
                self.upper_motor[0],
                self.upper_motor[1],
                upper_elbow[0],
                upper_elbow[1],
                self.params,
            )
            q2 = _elbow_q(
                self.lower_motor[0],
                self.lower_motor[1],
                lower_elbow[0],
                lower_elbow[1],
                self.params,
            )
            if q1 is not None and q2 is not None:
                solutions.append((q1, q2))

        if not solutions:
            raise ArmKinematicsError(f"末端点不可达：x={x:.4f}, y={y:.4f}")
        return tuple(solutions)
