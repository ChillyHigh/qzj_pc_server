# Planner 落地实现说明

本文档面向后续真正实现 `planner/` 的人。读完后应能知道：

- 比赛场地是什么样，机器人要完成什么任务。
- 为什么跨场要走 S 型路线。
- 取货、放货有哪些固定预设位置。
- `chassis`、`arm`、`funnel` 包分别提供哪些 path API。
- Planner 如何把这些 path 组织成可并行执行的 DAG。

Planner 只生成 `plan.DAG`，不连接 websocket，不读反馈，不下发命令。执行、调度、反馈门控由 `executor/` 完成。

## 1. 场地 Setting

场地使用世界坐标，单位 m：

- 场地中心是 `(0, 0)`。
- `x` 轴从取货区指向放置区，取货区在左侧 `x<0`，放置区在右侧 `x>0`。
- `y` 轴从下侧指向上侧。
- 规划可用边界按 `x in [-2.0, 2.0]`、`y in [-1.0, 1.0]` 处理。
- 底盘 `x/y/yaw` 的对外契约是四主动轮对角线交点，不是底盘几何中心。
- 四主动轮对角线交点相对底盘几何中心为 `(-0.035, 0.0)`，所以做碰撞和画图时要换算到底盘几何中心。

场地里有两个中间障碍柱：

| 障碍物 | 坐标 | 半径 | 高度 |
|---|---|---:|---:|
| 左障碍 | `(-1.000, 0.000)` | `0.051` | `0.500` |
| 右障碍 | `(1.000, 0.000)` | `0.051` | `0.500` |

规则要求货物跨场时绕开障碍物，不能直接从两个障碍之间按直线穿过去。当前实现把跨场动作收束为一条从左下到右上的 S 型路径。

`chassis.s_cross(...)` 只接受：

```text
start.x < -1.0 and start.y < 0.0
end.x   >  1.0 and end.y   > 0.0
```

也就是说，比赛主流程应先在取货侧从上往下取货，最后位于 `x<0, y<0` 附近，再通过 S 型路线跨到 `x>0, y>0` 的放置侧。

## 2. 比赛任务

赛前抽签给两个列表：

```python
pickup_assignment = [pickup1_bean, pickup2_bean, pickup3_bean]
drop_assignment = [box_at_pos4, box_at_pos5, box_at_pos6, box_at_pos7, box_at_pos8]
```

`pickup_assignment` 表示 1、2、3 号取货位分别是什么货物。货物编号：

| 编号 | 货物 |
|---:|---|
| 1 | 黄豆 |
| 2 | 绿豆 |
| 3 | 白芸豆 |

`drop_assignment` 表示 4-8 号放置位分别贴了什么编号的箱子。Planner 要从这两个输入反解：

```text
bean -> pickup_position
bean -> drop_position
```

例子：

```text
pickup_assignment = [3, 1, 2]
drop_assignment   = [4, 1, 2, 3, 5]
```

含义：

| 货物 | 取货位 | 目标放置位 |
|---|---:|---:|
| 黄豆 1 | 2 | 5 |
| 绿豆 2 | 3 | 6 |
| 白芸豆 3 | 1 | 7 |

规划第一版不做复杂重规划。目标是一趟完成：取三种货物，跨场，放三种货物，最后停到放置区侧。

## 3. 机器人几何和控制维度

底盘简化几何：

| 参数 | 值 |
|---|---:|
| 底盘长度 | `0.360` |
| 底盘宽度 | `0.670` |
| 坐标原点 | 四主动轮对角线交点 |
| 几何中心相对 drive center | `(0.035, 0.0)` |

五连杆和夹爪：

- Arm path 输出维度固定为 `(h, q1, q2, gripper_yaw, gripper_opening)`。
- `q1/q2` 使用 `[0, 2pi]`，IK 使用肘朝外分支。
- `EndEffectorState` 传入的是 arm 局部笛卡尔状态：

```python
(x, y, gripper_yaw, h, gripper_opening)
```

这里 `x/y` 是末端位置，不是 IK 后的 q。`gripper_yaw` 是相对底盘的夹爪方向。

Flags：

- flags 只控制上漏斗、下漏斗开关。
- 没有 gripper flag。
- gripper yaw/opening 都属于 arm path 的舵机目标角。

## 4. 预设配置结构

当前 Planner 预设全部在 `planner/config.py` 中直接写值，不定义 `*Preset` 类。

统一姿态 tuple：

```python
Pose = tuple[float, float, float, float, float, float, float]

(
    drive_x,
    drive_y,
    drive_yaw,
    arm_x,
    arm_y,
    gripper_yaw,
    h,
)
```

下标常量：

```python
DRIVE_X = 0
DRIVE_Y = 1
DRIVE_YAW = 2
ARM_X = 3
ARM_Y = 4
GRIPPER_YAW = 5
H = 6
```

目标箱矩形：

```python
TARGET_RECTS: dict[int, TargetRect]
```

其中：

```python
TargetRect = ((center_x, center_y), (half_x, half_y))
```

取货姿态：

```python
PICKUP_POSES: dict[int, Pose]
```

放置姿态：

```python
DROP_POSES: dict[int, dict[DropCarrier, Pose]]
```

第二层 key 固定为：

```text
upper_funnel
lower_funnel
gripper
```

漏斗还需要边缘对齐点：

```python
FUNNEL_DROP_BOX_EDGE_POINTS: dict[int, dict[DropCarrier, tuple[float, float]]]
```

这个点只用于校验和说明漏斗对齐逻辑，不是运动姿态本身。

## 5. 固定取货位

取货区有 3 个固定位置：

| 取货位 | 箱中心 | 半尺寸 |
|---:|---|---|
| 1 | `(-1.855, 0.500)` | `(0.105, 0.150)` |
| 2 | `(-1.855, -0.500)` | `(0.105, 0.150)` |
| 3 | `(-1.600, 0.000)` | `(0.105, 0.150)` |

当前固定取货停车姿态：

| 取货位 | drive pose `(x, y, yaw)` | arm `(x, y)` | `gripper_yaw` | 说明 |
|---:|---|---|---:|---|
| 1 | `(-1.480, 0.530, 0)` | `(-0.300, -0.030)` | `0` | 避开漏斗侧向包络与 3 号取货箱干涉 |
| 2 | `(-1.480, -0.530, 0)` | `(-0.300, 0.030)` | `0` | 避开漏斗侧向包络与 3 号取货箱干涉 |
| 3 | `(-1.300, 0.000, 0)` | `(-0.225, 0.000)` | `0` | 底盘左边界与 3 号箱右边界间距 `0.050m` |

这些点保证 arm 目标对准对应箱中心。3 号点按底盘左边界与箱体右边界 `0.050m` 间距设置。

## 6. 固定放置位

放置区有 5 个固定位置：

| 放置位 | 箱中心 | 半尺寸 | 朝向 |
|---:|---|---|---|
| 4 | `(1.640, 0.875)` | `(0.150, 0.105)` | 横向 |
| 5 | `(1.875, 0.400)` | `(0.105, 0.150)` | 竖向 |
| 6 | `(1.875, 0.000)` | `(0.105, 0.150)` | 竖向 |
| 7 | `(1.875, -0.400)` | `(0.105, 0.150)` | 竖向 |
| 8 | `(1.640, -0.875)` | `(0.150, 0.105)` | 横向 |

### 6.1 漏斗放置姿态

漏斗放置的语义：

- 箱子中心落在底盘几何中心坐标系的 `x=0.070` 线上。
- 上漏斗对齐边缘 `y=+0.350`。
- 下漏斗对齐边缘 `y=-0.350`。
- 4 下漏斗和 8 上漏斗对齐短边，避免底盘压箱。

| 放置位 | 载具 | 对齐边缘世界点 | drive pose `(x, y, yaw)` |
|---:|---|---|---|
| 4 | upper_funnel | `(1.640, 0.770)` | `(1.535, 0.420, 0)` |
| 4 | lower_funnel | `(1.490, 0.875)` | `(1.140, 0.770, pi/2)` |
| 5 | upper_funnel | `(1.770, 0.400)` | `(1.420, 0.505, 3π/2)` |
| 5 | lower_funnel | `(1.770, 0.400)` | `(1.420, 0.295, pi/2)` |
| 6 | upper_funnel | `(1.770, 0.000)` | `(1.420, 0.105, 3π/2)` |
| 6 | lower_funnel | `(1.770, 0.000)` | `(1.420, -0.105, pi/2)` |
| 7 | upper_funnel | `(1.770, -0.400)` | `(1.420, -0.295, 3π/2)` |
| 7 | lower_funnel | `(1.770, -0.400)` | `(1.420, -0.505, pi/2)` |
| 8 | upper_funnel | `(1.490, -0.875)` | `(1.140, -0.770, 3π/2)` |
| 8 | lower_funnel | `(1.640, -0.770)` | `(1.535, -0.420, 0)` |

### 6.2 夹爪放置姿态

夹爪放置的语义：

- 5/6/7 号箱中心落在 arm 局部 `(-0.300, 0.000)`；4/8 号斜角放置为避开箱体，箱中心落在 arm 局部 `(-0.320, 0.000)`。
- 夹爪长边对齐箱子长边。
- 夹爪长边是无向轴，`gripper_yaw=pi` 和 `0` 等价时取较小值。

| 放置位 | drive pose `(x, y, yaw)` | arm `(x, y)` | `gripper_yaw` |
|---:|---|---|---:|
| 4 | `(1.360693, 0.595693, 225deg)` | `(-0.320, 0.000)` | `45deg` |
| 5 | `(1.500000, 0.400000, 180deg)` | `(-0.300, 0.000)` | `0deg` |
| 6 | `(1.500000, 0.000000, 180deg)` | `(-0.300, 0.000)` | `0deg` |
| 7 | `(1.500000, -0.400000, 180deg)` | `(-0.300, 0.000)` | `0deg` |
| 8 | `(1.360693, -0.595693, 135deg)` | `(-0.320, 0.000)` | `135deg` |

当前点位图：

```text
docs/planner_points_overview.png
```

## 7. Path API

Planner 不直接创建轨迹数组，而是调用执行层已经稳定的 path API。

### 7.1 Chassis API

```python
chassis.move(
    start: tuple[float, float, float],
end: tuple[float, float, float],
speed_scale: float,
) -> AbstractGeometricPath
```

用于同侧短距离移动，例如：

- 起始区到取货侧预备点。
- 取货 1 号位 -> 取货 2 号位-> 取货 3 号位。
- 跨场后在放置区内移动。

```python
chassis.s_cross(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    speed_scale: float,
) -> AbstractGeometricPath
```

用于左下到右上的跨场动作。它内部使用 `chassis/config.py` 中的 S 型控制点绕开两个障碍物。输入不满足左下到右上时会明确失败，不做 silent fallback。

### 7.2 Arm API

```python
arm.direct(
    start: tuple[float, float, float, float, float],
end: tuple[float, float, float, float, float],
speed_scale: float,
) -> AbstractGeometricPath
```

语义：

```text
(x, y, gripper_yaw, h, gripper_opening)
```

`x/y/gripper_yaw` 是末端笛卡尔目标，函数内部执行 IK，输出 path 的 q 是：

```text
(h, q1, q2, gripper_yaw_servo_target, gripper_opening)
```

跨五连杆半平面时，`arm.move` 会插入预计算的奇异点附近 joint waypoints。

```python
arm.prepare_pick(
    start: tuple[float, float, float, float, float],
    target: tuple[float, float, float, float, float],
    moving_h: float,
    speed_scale: float,
) -> AbstractGeometricPath
```

用于取货前预备：

1. 从当前末端状态抬到 `moving_h`。
2. 在移动高度平移到目标上方。
3. 下降到目标取货高度。

```python
arm.do_pick(
    start: tuple[float, float, float, float, float],
end_opening: float,
speed_scale: float,
) -> AbstractGeometricPath
```

输入是 joint-space 状态：

```text
(h, q1, q2, gripper_yaw, gripper_opening)
```

用于“一边上升，一边闭合夹爪”。它保持 `q1/q2/gripper_yaw`，把 `h` 增加 `arm.config.GRIP_LIFT_H`，同时把 `gripper_opening` 改到目标值。

```python
arm.set_gripper(
    state: tuple[float, float, float, float, float],
    opening: float,
) -> AbstractGeometricPath
```

用于只开合夹爪，不改变末端位置、夹爪 yaw 和升降高度。`state` 是 EndEffectorState：

```text
(x, y, gripper_yaw, h, current_opening)
```

Planner 释放夹爪中的货物或预先张开夹爪时，应优先调用这个 API，而不是用 `arm.move(state, state_with_new_opening, ...)` 表达纯开合动作。

### 7.3 Funnel API

```python
funnel.upper(open: bool) -> AbstractGeometricPath
funnel.lower(open: bool) -> AbstractGeometricPath
funnel.set(upper_open: bool, lower_open: bool) -> AbstractGeometricPath
funnel.close_all() -> AbstractGeometricPath
```

打开漏斗的 flags path duration 使用 `funnel/config.py` 中的 `OPEN_DURATION`，当前为 `2.0s`；关闭漏斗仍为 `0.0s`。flags action 仍然是 `ActionNode(kind="flags")`，会通过 executor 写入状态。flags 只有上漏斗和下漏斗两个 bit。

## 8. DAG 如何合理并行

DAG 节点：

```python
start = AbstractNode("start")

move = ActionNode(
    name="move_to_pick_1",
    deps=[start],
    kind="chassis",
    path=chassis_path,
)

wait = WaitNode(
    name="wait_pick_1_pose",
    deps=[start],
    target=chassis_reached(...),
    timeout=2.0,
)

arm_prepose = ActionNode(
    name="arm_prepose_pick_1",
    deps=[wait],
    kind="arm",
    path=arm_path,
)
```

关键点：

- `ActionNode.kind` 是资源锁，`chassis`、`arm`、`flags` 三种资源可并行。
- 同一 kind 的 ActionNode 可以同时解锁，但 executor 会互斥执行。
- `WaitNode` 只看反馈，不占资源。
- 底盘到位等待通常应和底盘移动从同一个 dep 启动，而不是依赖底盘 Action 完成。

正确门控：

```text
start -> chassis_move
start -> wait_chassis_reached -> arm_insert
```

含义是：底盘开始移动后，WaitNode 同时开始看反馈；反馈到位后 arm 可以开始。此时 chassis action 是否还在运行，由 executor 的 kind 锁和实际动作完成状态共同处理。

不推荐：

```text
start -> chassis_move -> wait_chassis_reached -> arm_insert
```

除非语义明确是“底盘 path 完全结束后才开始看反馈”。精确取放通常不是这个语义。

## 9. 推荐比赛流程

第一版建议采用固定流程，少做候选搜索：

1. 解析抽签：
   - `bean -> pickup_pos`
   - `bean -> drop_pos`
2. 取货侧按 `1 -> 3 -> 2` 号位顺序取。
3. 对每个取货位：
   - `chassis.direct` 到 `PICKUP_POSES[pos]`。
   - `WaitNode` 等底盘快到取货点了就prepare，不要等到了再prepare。
   - `arm.prepare_pick` 到目标取货姿态。
   - `arm.grip_lift` 完成夹取和上升。
   - 根据容器分配，把货物留在 gripper，或移动到对应漏斗并释放。
4. 完成取货后，确保底盘位于左下跨场入口。
5. 使用 `chassis.s_cross` 从左下跨到右上，绕过两个障碍。
6. 放置侧根据目标位依次放货：
   - 如果容器是 `upper_funnel`，到 `DROP_POSES[pos]["upper_funnel"]`，Wait 到位后打开上漏斗。
   - 如果容器是 `lower_funnel`，到 `DROP_POSES[pos]["lower_funnel"]`，Wait 到位后打开下漏斗。
   - 如果容器是 `gripper`，到 `DROP_POSES[pos]["gripper"]`，Wait 到位后用 `arm.move` 到放置姿态，再张开夹爪。
7. 关闭漏斗 flags。
8. 底盘移动到结束停车点，确保可移动部分全部在放置区。

## 10. 容器分配建议

当前机器人可用三个载具：

```text
upper_funnel
lower_funnel
gripper
```

需要将三种货物分别分配给三个载具，保证在一趟跨场完成。

候选数量不要一开始做太大。建议第一版只枚举：

- 取货顺序：`1->3->2`
- 容器分配：三种货物到三个载具的排列。
- 放货顺序：按放置位从上到下，或按载具动作减少底盘移动。

每个候选生成 DAG 后估算完成时间，取最短。估算只用于 planner 内部选择，不传给 executor 当作运行依据。

## 11. 从 Pose 生成 motion

从 `planner/config.py` 取到 `Pose` 后，常用拆法：

```python
drive = (
    pose[config.DRIVE_X],
    pose[config.DRIVE_Y],
    pose[config.DRIVE_YAW],
)

arm_target = (
    pose[config.ARM_X],
    pose[config.ARM_Y],
    pose[config.GRIPPER_YAW],
    pose[config.H],
    gripper_opening,
)
```

取货：

```python
pickup_pose = config.PICKUP_POSES[pickup_pos]
drive_target = pickup_pose[:3]

chassis_path = chassis.move(current_drive, drive_target, speed_scale=0.8)
arm_path = arm.prepare_pick(
    current_arm_cartesian,
    (
        pickup_pose[config.ARM_X],
        pickup_pose[config.ARM_Y],
        pickup_pose[config.GRIPPER_YAW],
        pickup_pose[config.H],
        open_angle,
    ),
    moving_h=0.30,
    speed_scale=0.8,
)
```

漏斗放货：

```python
drop_pose = config.DROP_POSES[drop_pos]["upper_funnel"]
chassis_path = chassis.move(current_drive, drop_pose[:3], speed_scale=0.8)
open_path = funnel.upper(True)
close_path = funnel.upper(False)
```

夹爪放货：

```python
drop_pose = config.DROP_POSES[drop_pos]["gripper"]
drop_state = (
    drop_pose[config.ARM_X],
    drop_pose[config.ARM_Y],
    drop_pose[config.GRIPPER_YAW],
    drop_pose[config.H],
    closed_angle,
)
arm_path = arm.direct(
    current_arm_cartesian,
    drop_state,
    speed_scale=0.8,
)
release_path = arm.set_gripper(drop_state, open_angle)
```

注意：`arm.move` 的 start/end 都是笛卡尔末端状态，不是 joint q。纯夹爪开合使用 `arm.set_gripper(...)`，不要用 `arm.move` 伪装成开合动作。

## 12. 输出前校验

Planner 输出 DAG 前至少校验：

- DAG 非空。
- 节点 name 唯一。
- deps 都在 DAG 内。
- DAG 无环。
- 每个 ActionNode 的 path 不为 None。
- path 维度匹配 kind：
  - chassis: 3
  - arm: 5
  - flags: 1
- 关键取放动作前有底盘到位 WaitNode。
- 精确取放时底盘外廓不压目标箱、其他箱、障碍、围栏。
- arm 目标可 IK。
- `s_cross` 只用于左下到右上跨场。

这些校验失败时应明确抛错，不做 silent fallback。

## 13. 当前不做的事

不做：

- 动态连续搜索停车点。
- 运行中失败后重新规划。

当前预设位置已经画在：

```text
docs/planner_points_overview.png
```

后续修改预设时，应同步：

- `planner/config.py`
- `tests/test_planner_config.py`
- `docs/Planner落地实现说明.md`
- `docs/planner_points_overview.png`
