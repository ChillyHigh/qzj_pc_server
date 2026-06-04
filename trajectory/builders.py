from __future__ import annotations

from typing import Any

import numpy as np

from arm import FiveBarKinematics

from .types import Waypoint


class ArmPath:
    """把 arm 末端路径转换成 TOPPRA 航点。

    该类只负责末端离散和连续 IK，不做时间参数化。输出 q 格式为
    `(h, q1, q2, gripper_yaw, gripper_opening)`。
    """

    def __init__(self, kinematics: FiveBarKinematics, *, step: float = 0.005) -> None:
        """创建 arm 路径构造器。

        Args:
            kinematics: 五连杆正逆运动学模型。
            step: 末端直线插值步长，单位 m。
        """

        self.kinematics = kinematics
        self.step = step
        self._source_id = 0

    def line(
        self,
        start_xy: tuple[float, float],
        end_xy: tuple[float, float],
        *,
        initial_q: tuple[float, float] | None = None,
        h: float = 0.0,
        gripper_yaw: float = 0.0,
        gripper_opening: float = 0.0,
        speed_scale: float = 1.0,
        meta: dict[str, Any] | None = None,
    ) -> list[Waypoint]:
        """生成一段末端直线对应的 arm 航点。

        Args:
            start_xy: 起点末端坐标，单位 m。
            end_xy: 终点末端坐标，单位 m。
            initial_q: 起点附近参考 `(q1, q2)`，用于选择连续 IK 解。
            h: 升降轴目标值。
            gripper_yaw: 夹爪 yaw，rad。
            gripper_opening: 夹爪开合角/开合量，按协议定义。
            speed_scale: 该段速度倍率。
            meta: 透传给轨迹采样点的动作信息。
        """

        dist = float(np.hypot(end_xy[0] - start_xy[0], end_xy[1] - start_xy[1]))
        count = max(1, int(np.ceil(dist / self.step))) + 1
        alphas = np.linspace(0.0, 1.0, count)
        start = np.asarray(start_xy, dtype=float)
        end = np.asarray(end_xy, dtype=float)
        xy = start[None, :] * (1.0 - alphas[:, None]) + end[None, :] * alphas[:, None]

        q12 = self.kinematics.ik_path(xy, initial_q=initial_q)
        source_id = self._source_id
        self._source_id += 1
        return [
            Waypoint(
                q=(h, float(q1), float(q2), gripper_yaw, gripper_opening),
                speed_scale=speed_scale,
                meta=dict(meta or {}),
                source_kind="segment",
                source_id=source_id,
            )
            for q1, q2 in q12
        ]
