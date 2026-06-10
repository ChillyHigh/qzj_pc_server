# chassis / arm 包重构方案

## 设计原则

1. **类型驱动，不堆 shim**：接口变更就迁移调用方，不加兼容层。
2. **类型注释即契约**：`ChassisWaypoint` 字段类型已保证 3 维 float，不再写运行时维度/NaN 检查。bounds（q_min/q_max）保留——类型表达不了值域。
3. **统一为 `PlannedPath`**：删除 `ArmServoPath` 和 `_FlagsPath`，所有路径对象都是 `PlannedPath`。`PlannedPath` 是具体类（dataclass），不是 Protocol——`ActionNode.path` 类型就是它，不需要 structural subtyping。

---

## 1. `plan/types.py` — 新增 `PlannedPath`，改 `ActionNode.path`

### 新增 `PlannedPath`

```python
from dataclasses import dataclass, field
from typing import Callable
import numpy as np


@dataclass(frozen=True, slots=True)
class PlannedPath:
    """planner ↔ executor 路径契约。"""
    _sampler: Callable[[np.ndarray, int], np.ndarray] = field(repr=False)
    duration: float

    def __call__(self, path_positions, order: int = 0) -> np.ndarray:
        return self._sampler(np.asarray(path_positions, dtype=float), order)
```

- `_sampler(t, order)`：`t` 是 `np.ndarray`（标量或数组），`order=0` 返回 q，`order=1` 返回 dq。
- `duration` 总是 float。
- frozen + slots，不可变，无额外运行时开销。

### 修改 `ActionNode.path`

```python
# 旧
from toppra.interpolator import AbstractGeometricPath  # 删掉 TYPE_CHECKING 块
path: AbstractGeometricPath | None = None

# 新
path: PlannedPath | None = None
```

`AbstractGeometricPath` 的 import 全部删除。

---

## 2. `trajectory/types.py` — `Waypoint.q` 改为 property

```python
@dataclass(frozen=True, slots=True)
class Waypoint:
    speed_scale: float = 1.0
    meta: dict[str, Any] = field(default_factory=dict)
    source_kind: str = "single"
    source_id: int | None = None
    blend_single: bool = False

    @property
    def q(self) -> tuple[float, ...]:
        raise NotImplementedError
```

删 `q: tuple[float, ...]` 构造参数。`densify_and_smooth` 等下游读 `waypoint.q` 不变。

---

## 3. `trajectory/toppra_planner.py` — 保持不变，`_check_waypoints` 精简

`ToppraPlanner.plan()` 保持返回 `AbstractGeometricPath`（toppra 库原生类型）。基类不引入 `PlannedPath` 依赖——`PlannedPath` 的包装在子类层完成。

```python
class ToppraPlanner:
    def plan(self, waypoints: list[Waypoint], *, max_step: float = 0.05) -> AbstractGeometricPath:
        ...  # 不变
        trajectory = instance.compute_trajectory(0.0, 0.0)
        if trajectory is None:
            raise TrajectoryError("TOPPRA 未能生成可行轨迹。")
        return trajectory
```

### `_check_waypoints` 只保留类型表达不了的

```python
def _check_waypoints(self, waypoints: list[Waypoint]) -> None:
    expected_dim = len(waypoints[0].q)
    if expected_dim != len(self.vlim) or expected_dim != len(self.alim):
        raise TrajectoryError("航点维度与速度/加速度限制维度不一致。")
    if self.q_min is not None or self.q_max is not None:
        for idx, waypoint in enumerate(waypoints):
            q = np.asarray(waypoint.q, dtype=float)
            if self.q_min is not None and np.any(q < self.q_min):
                raise TrajectoryError(f"航点[{idx}] 小于 q_min。")
            if self.q_max is not None and np.any(q > self.q_max):
                raise TrajectoryError(f"航点[{idx}] 大于 q_max。")
```

删除 per-waypoint 维度检查和 NaN 检查（Waypoint 子类类型已保证）。

---

## 4. `chassis/` — 拆两个文件

### 4.1 `chassis/toppra_planner.py`（重写）

```python
from __future__ import annotations
from dataclasses import dataclass

from toppra.interpolator import AbstractGeometricPath

from plan.types import PlannedPath
from trajectory import ToppraPlanner, Waypoint
from . import config


class ChassisPathError(ValueError):
    """底盘 path 生成失败。"""


@dataclass(frozen=True, slots=True)
class ChassisWaypoint(Waypoint):
    x: float
    y: float
    yaw: float

    @property
    def q(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.yaw)


class ChassisToppraPlanner(ToppraPlanner):
    def __init__(self) -> None:
        super().__init__(vlim=config.V_LIMIT, alim=config.A_LIMIT)

    def plan(
        self, waypoints: list[ChassisWaypoint], *, max_step: float = 0.05
    ) -> PlannedPath:
        trajectory: AbstractGeometricPath = super().plan(waypoints, max_step=max_step)
        return PlannedPath(trajectory, float(trajectory.duration))
```

> `super().plan()` 返回 `AbstractGeometricPath`，其 `__call__(t, order)` 签名与 `PlannedPath._sampler` 一致，直接传入；包装后返回 `PlannedPath`。

无 `_check_chassis_waypoints`。`ChassisWaypoint` 的三个 `float` 字段已足够。

### 4.2 `chassis/path_generator.py`（新建）

从旧 `chassis/toppra_planner.py` 搬入 `move`、`direct`、`s_cross`：

- `Waypoint(q=(x,y,yaw), ...)` → `ChassisWaypoint(x=x, y=y, yaw=yaw, ...)`
- 返回类型 → `PlannedPath`
- `_planner()` → `ChassisToppraPlanner()`
- 工具函数 `_shortest_yaw_delta`、`_continuous_yaw_points`、`_unwrap_end_yaw` 一并搬入

### 4.3 `chassis/__init__.py`

```python
from .geometry import is_drive_pose_colliding, plan_avoidance_path, validate_chassis_path
from .path_generator import direct, move, s_cross
from .toppra_planner import ChassisPathError, ChassisToppraPlanner, ChassisWaypoint

__all__ = [
    "ChassisPathError", "ChassisToppraPlanner", "ChassisWaypoint",
    "direct", "is_drive_pose_colliding", "move",
    "plan_avoidance_path", "s_cross", "validate_chassis_path",
]
```

### 4.4 `chassis/geometry.py`、`chassis/config.py` — 不改

---

## 5. `arm/` — 删除 `ArmServoPath`，改为工厂函数返回 `PlannedPath`

### 5.1 `arm/toppra_planner.py`（重写）

#### ArmWaypoint

```python
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
```

#### 删除 `ArmServoPath`，改为 `_create_arm_path` 工厂函数

`ArmServoPath.__init__` 和 `__call__` 的全部逻辑搬进一个闭包，返回 `PlannedPath`。
`motion_path` 参数接收 `AbstractGeometricPath | None`（来自 `super().plan()` 的 toppra 原生轨迹）。

```python
def _create_arm_path(
    motion_path: AbstractGeometricPath | None,
    motion_waypoints: np.ndarray,
    servo_waypoints: np.ndarray,
    servo_v_limit: np.ndarray,
) -> PlannedPath:
    # —— 计算 waypoint_times（原 ArmServoPath.__init__）——
    motion_duration = 0.0 if motion_path is None else float(motion_path.duration)
    if motion_path is None:
        waypoint_times = np.zeros(len(motion_waypoints), dtype=float)
    else:
        t_grid = np.linspace(0.0, motion_duration, max(2, len(motion_waypoints) * 20))
        motion_samples = np.asarray(motion_path(t_grid, order=0), dtype=float)
        wt = []
        for wq in motion_waypoints:
            idx = int(np.argmin(np.linalg.norm(motion_samples - wq, axis=1)))
            wt.append(float(t_grid[idx]))
        waypoint_times = np.maximum.accumulate(np.asarray(wt, dtype=float))
        waypoint_times[0] = 0.0
        waypoint_times[-1] = motion_duration

    # —— 计算 servo 完成时间（原 _servo_completion_time）——
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

    # —— sampler（原 ArmServoPath.__call__）——
    def sampler(t: np.ndarray, order: int) -> np.ndarray:
        scalar = t.ndim == 0
        if motion_path is None:
            motion = np.zeros((1 if scalar else len(t), 3), dtype=float)
            if order == 0:
                motion[:] = motion_waypoints[-1]
        else:
            motion_t = np.clip(t, 0.0, motion_duration)
            motion = np.asarray(motion_path(motion_t, order=order), dtype=float)
            if order in (1, 2):
                stopped = t > motion_duration
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
```

> `_move_towards` 保留为模块级函数，逻辑不变。

#### `ArmToppraPlanner`

```python
class ArmToppraPlanner(ToppraPlanner):
    def __init__(self) -> None:
        super().__init__(
            vlim=config.MOTION_V_LIMIT,
            alim=config.MOTION_A_LIMIT,
            q_min=config.Q_MIN_LIMIT[:3],
            q_max=config.Q_MAX_LIMIT[:3],
        )

    def plan(
        self, waypoints: list[ArmWaypoint], *, max_step: float = 0.05
    ) -> PlannedPath:
        if len(waypoints) < 2:
            raise TrajectoryError("Arm TOPPRA 至少需要 2 个航点。")

        motion_waypoints = [
            Waypoint(  # 基类 Waypoint，只用 q[:3]
                q=tuple(float(v) for v in waypoint.q[:3]),
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
```

无 `_check_arm_bounds`——bounds 检查通过 `super().__init__(q_min=..., q_max=...)` 交给基类 `_check_waypoints`，不重复实现。

#### `plan_joint_waypoints`

```python
def plan_joint_waypoints(waypoints: list[ArmWaypoint]) -> PlannedPath:
    return ArmToppraPlanner().plan(waypoints)
```

### 5.2 `arm/path_generator.py`

- `Waypoint(q=(h, q1, q2, gy, go), ...)` → `ArmWaypoint(h=h, q1=q1, q2=q2, gripper_yaw=gy, gripper_opening=go, ...)`
- 返回类型 `AbstractGeometricPath` → `PlannedPath`
- import 从 `arm.toppra_planner` 取 `ArmWaypoint`

### 5.3 `arm/__init__.py`

```python
from .five_bar import FiveBarKinematics
from .path_generator import do_pick, move, set_gripper
from .toppra_planner import ArmPathError, ArmToppraPlanner, ArmWaypoint
from .types import ArmKinematicsError, FiveBarParams

__all__ = [
    "ArmKinematicsError", "ArmPathError", "ArmToppraPlanner", "ArmWaypoint",
    "FiveBarKinematics", "FiveBarParams",
    "do_pick", "move", "set_gripper",
]
```

### 5.4 `arm/config.py`、`arm/types.py`、`arm/five_bar.py` — 不改

---

## 6. `funnel/__init__.py` — 删除 `_FlagsPath`，函数直接返回 `PlannedPath`

```python
from plan.types import PlannedPath


def set(upper_open: bool, lower_open: bool) -> PlannedPath:
    flags = 0
    if upper_open:
        flags |= FLAG_UPPER_FUNNEL_OPEN
    if lower_open:
        flags |= FLAG_LOWER_FUNNEL_OPEN
    duration = config.OPEN_DURATION if flags != 0 else 0.0

    def sampler(t: np.ndarray, order: int) -> np.ndarray:
        if order == 0:
            value = float(flags)
        elif order == 1:
            value = 0.0
        else:
            return np.zeros_like(t, dtype=float)  # order>=2 返回 0
        if t.ndim == 0:
            return np.array([value], dtype=float)
        return np.full((len(t), 1), value, dtype=float)

    return PlannedPath(sampler, duration)
```

`upper`、`lower`、`close_all` 改为调用 `set` 并返回 `PlannedPath`。删除 `_FlagsPath` 类、`FunnelPathError`（flags 非法由 `set` 内部 assert/raise `ValueError` 即可）。

---

## 7. `plan/dag.py` — 删除冗余运行时检查

`_validate_action` 中：

- `hasattr(node.path, "duration")` 删除——`PlannedPath` 是具体类，必定有 `duration`。
- `not np.isfinite(duration)` 保留——`float` 不保证 finite。
- 维度检查保留——`KIND_DIMS` 校验是业务逻辑，类型表达不了。
- NaN 检查删除——`PlannedPath._sampler` 返回的 `np.ndarray` 由各 sampler 保证。

---

## 8. 不改的文件

| 文件 | 原因 |
|------|------|
| `chassis/geometry.py` | 只依赖 `plan.setting` + `chassis.config`，不涉及 waypoint/path 类型 |
| `chassis/config.py` | 纯常量 |
| `arm/config.py` | 纯常量 |
| `arm/types.py` | FiveBarParams + ArmKinematicsError，与 waypoint/path 无关 |
| `arm/five_bar.py` | 只做运动学，不涉及 waypoint/path |
| `plan/setting.py` | 场地常量 |
| `planner/planner.py` | 通过 `chassis.*` / `arm.*` API 获取路径，类型自动变为 `PlannedPath` |
| `planner/config.py` | 纯常量 |
| `executor/executor.py` | 只访问 `path.duration` / `path(t, order)` |
| `executor/mixer.py` | 只访问 `path(t, order)` |
| `trajectory/densify.py` | 只读 `waypoint.q`，不构造 Waypoint |
| `trajectory/smoothing.py` | 同上 |

---

## 9. 文件变更总览

| 操作 | 文件 | 说明 |
|------|------|------|
| 修改 | `plan/types.py` | 新增 `PlannedPath` dataclass；`ActionNode.path` 类型改为 `PlannedPath`；删 `AbstractGeometricPath` import |
| 修改 | `trajectory/types.py` | `Waypoint.q` 变 property |
| 修改 | `trajectory/toppra_planner.py` | `plan()` 保持返回 `AbstractGeometricPath`；`_check_waypoints` 只保留 vlim/alim 一致性 + bounds |
| **重写** | `chassis/toppra_planner.py` | `ChassisWaypoint` + `ChassisToppraPlanner`；移走 path_generator 函数 |
| **新建** | `chassis/path_generator.py` | `move`/`direct`/`s_cross` |
| 修改 | `chassis/__init__.py` | 导出更新 |
| **重写** | `arm/toppra_planner.py` | `ArmWaypoint`；`ArmServoPath` → `_create_arm_path`；bounds 通过基类 `__init__` 传入 |
| 修改 | `arm/path_generator.py` | `Waypoint` → `ArmWaypoint`；返回类型 `PlannedPath` |
| 修改 | `arm/__init__.py` | 导出新增 `ArmWaypoint` |
| 修改 | `funnel/__init__.py` | 删除 `_FlagsPath` + `FunnelPathError`；函数返回 `PlannedPath` |
| 修改 | `plan/dag.py` | 删 `hasattr` 检查 |

---

## 10. 类型流（重构后）

```
trajectory.Waypoint  (q 为 abstract property)
  ├── ChassisWaypoint(x, y, yaw)                         → q = (x, y, yaw)
  └── ArmWaypoint(h, q1, q2, gripper_yaw, gripper_opening) → q = (h, q1, q2, gy, go)

trajectory.ToppraPlanner  (plan() -> AbstractGeometricPath, 基类不感知 PlannedPath)
  ├── ChassisToppraPlanner  (super().plan() -> AbstractGeometricPath → 包装为 PlannedPath)
  └── ArmToppraPlanner      (super().plan() -> AbstractGeometricPath → _create_arm_path → PlannedPath)

PlannedPath  (dataclass, 唯一对外路径类型)
  .duration: float
  .__call__(path_positions, order=0) -> np.ndarray

ActionNode.path: PlannedPath | None
```

`AbstractGeometricPath` 只存在于 `trajectory` 和 `arm`/`chassis` 的 planner 子类内部，对外（`plan`/`executor`/`planner`）全部走 `PlannedPath`。
