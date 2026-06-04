# 一趟任务规划说明

本文档只描述规划层。控制、通信、仿真和机械建模放在其他模块中处理。

规划层要回答三个问题：

- 按什么顺序取三种豆子。
- 每种豆子通过上漏斗、下漏斗还是夹爪携带和放出。
- 到右侧目标箱时按什么顺序放，车身 yaw 取什么角度。
- 跨场 S 型移动过程中是否同步旋转 yaw。
- 取货区是否利用 3 个纸垛形成的凹陷位置，减少底盘平移。

## 1. 输入与输出

规划输入：

- `pickup_assignment`：取货区 1、2、3 号位分别是什么豆子。
- `drop_assignment`：放置区 4、5、6、7、8 号位分别是什么编号货箱。
- `motion_model`：动作耗时模型，用于估计底盘移动、旋转、升降、五连杆动作时间。
- `drop_pose_library`：每个放置位的可用停车姿态、yaw、可用容器。

规划输出：

- `container_assignment`：豆子到 `upper_funnel / lower_funnel / gripper` 的分配。
- `pickup_order`：取货位访问顺序。
- `drop_order`：放置位访问顺序。
- `drop_actions`：每个放置动作使用的容器、停车点、yaw、工具动作。
- `estimated_time`：调度后的预计总时间。

## 2. 编码约定

豆子编码沿用规则抽签说明：

- `1`：黄豆，目标为编号 1 的货箱。
- `2`：绿豆，目标为编号 2 的货箱。
- `3`：白芸豆，目标为编号 3 的货箱。

放置区位置编码：

- `4`：上侧横向箱。
- `5`：右侧上箱。
- `6`：右侧中箱。
- `7`：右侧下箱。
- `8`：下侧横向箱。

目标箱映射：

```text
黄豆 -> 编号 1 货箱
绿豆 -> 编号 2 货箱
白芸豆 -> 编号 3 货箱
```

规划时先把 `drop_assignment` 反解成：

```text
bean -> target_drop_position
```

## 3. 场地方向

场地坐标按当前 MuJoCo 约定理解：

- `x` 轴从左到右。
- `y` 轴从下到上。
- 取货区在左侧。
- 放置区在右侧。

从取货区移动到放置区的主跨场方向为：

```text
左下 -> 右上
```

这个方向对规划有两个影响：

- 跨场路径默认从取货区下侧切入 S 型通道，再从右上侧进入放置区。
- 右侧卸料默认按从上到下的扫描顺序生成候选，即优先考虑 `4/5 -> 6 -> 7/8`。

扫描顺序只是候选优先级，最终顺序由耗时评分决定。

## 4. 放置姿态模板

每个放置位需要定义一组候选姿态。规划层只读取模板，不在逻辑中写死几何细节。

姿态模板字段：

```python
DropPose(
    drop_pos: int,
    carrier: str,      # upper_funnel / lower_funnel / gripper
    x: float,
    y: float,
    yaw: float,
    h: float,
    q1: float,
    q2: float,
    cost_bias: float,
)
```

当前几何判断：

- `4` 和 `8` 是上下横向箱，通过漏斗放置时各自存在多种 yaw 候选。
- `5`、`6`、`7` 是右侧竖向箱，通过漏斗放置时也应按容器和 yaw 枚举候选。
- 夹爪取放的自由度较大，只要给定 yaw 下五连杆末端覆盖取/放点，且车体外廓安全，该姿态就是可行候选。
- 每个放置位应分别评估上漏斗、下漏斗、夹爪的可行姿态。

漏斗放置姿态应按“放置位 + 容器 + yaw”展开。以 4 号位为例，已知存在这些候选：

```text
drop_4:
  - upper_funnel, yaw = 0 deg
  - upper_funnel, yaw = -90 deg
  - lower_funnel, yaw = 180 deg
  - lower_funnel, yaw = 90 deg
  - gripper, yaw = any yaw that lets the arm endpoint cover the box

drop_5:
  - upper_funnel, yaw = candidate set
  - lower_funnel, yaw = candidate set
  - gripper, yaw = any yaw that lets the arm endpoint cover the box

drop_6:
  - upper_funnel, yaw = candidate set
  - lower_funnel, yaw = candidate set
  - gripper, yaw = any yaw that lets the arm endpoint cover the box

drop_7:
  - upper_funnel, yaw = candidate set
  - lower_funnel, yaw = candidate set
  - gripper, yaw = any yaw that lets the arm endpoint cover the box

drop_8:
  - upper_funnel, yaw = candidate set
  - lower_funnel, yaw = candidate set
  - gripper, yaw = any yaw that lets the arm endpoint cover the box
```

8 号位也应像 4 号位一样枚举四种漏斗放法，具体 yaw 由对称关系和实测卸料方向确认后写入姿态库。

`candidate set` 应由几何推导和实车/仿真验证共同确定。规划层只读取候选集，不在规划逻辑中写死具体角度。

### 4.1 夹爪取放可行性

夹爪动作不应只绑定固定停车点。

对某个取/放物点，夹爪候选姿态的判断方式：

```text
1. 枚举或采样车身 yaw。
2. 根据车身 yaw 和底盘候选位置，把目标点变换到机器人局部坐标。
3. 判断目标点是否处于五连杆末端可达区域。
4. 判断 q1/q2 是否在 0 到 180 度范围内。
5. 判断车体、连杆、夹爪与箱体/围栏/障碍物是否满足安全裕量。
6. 满足以上条件则生成一个 gripper DropPose 或 PickupPose。
```

这类候选可以很多，评分时由移动时间、旋转时间、连杆动作时间和风险代价共同筛选。

## 5. 容器分配

三种豆子分配到三个容器：

- `upper_funnel`
- `lower_funnel`
- `gripper`

规划层枚举全部 `3! = 6` 种分配。

每种分配都检查：

- 该豆子的目标放置位是否存在对应容器的 `DropPose`。
- 该放置姿态是否满足 yaw 和停车空间约束。
- 三个放置动作是否能组成合理的右侧访问顺序。

容器分配的核心依据是右侧目标位置和卸料姿态。取货区位置只影响左侧取货顺序和装载耗时。

## 6. 取货顺序

取货区的 3 个纸垛呈凸字形。取物时主要考虑两个凹陷停靠区：

- `pickup_station_12_recess`：位于 1 号位和 2 号位形成的凹陷处，用于连续取 1、2 两个取货位。
- `pickup_station_23_recess`：位于 2 号位和 3 号位形成的凹陷处，用于连续取 2、3 两个取货位。

因此取货规划应以“凹陷停靠区 + yaw 旋转 + 五连杆覆盖”建模。

取货姿态模板：

```python
PickupPose(
    pickup_pos: int,
    x: float,
    y: float,
    yaw: float,
    h: float,
    q1: float,
    q2: float,
    station_id: str,
    cost_bias: float,
)
```

建议先定义这些取货停靠点：

```text
pickup_station_12_recess:
  - 位于 1 号位和 2 号位形成的凹陷处
  - 主要服务 1 号位和 2 号位
  - 同一底盘位置下枚举多个 yaw，减少平移

pickup_station_23_recess:
  - 位于 2 号位和 3 号位形成的凹陷处
  - 主要服务 2 号位和 3 号位
  - 同一底盘位置下枚举多个 yaw，减少平移
```

对每个取货位，规划器生成多个 `PickupPose` 候选：

```text
1. 枚举可用 pickup_station。
2. 在每个 station 上枚举 yaw。
3. 判断五连杆末端是否覆盖取货点。
4. 判断车体是否与纸垛、货箱、围栏保持安全裕量。
5. 满足条件则保留候选。
```

确定容器分配后，取货顺序按以下原则生成候选：

- 夹爪携带的豆子倾向最后取。
- 两个漏斗对应的豆子先取并装入漏斗。
- 在两个漏斗豆之间，优先选择共用取货停靠点、减少底盘平移的顺序。
- 对 1 号位和 2 号位，优先尝试 `pickup_station_12_recess` 连续取货。
- 对 2 号位和 3 号位，优先尝试 `pickup_station_23_recess` 连续取货。
- 对 1 号位和 3 号位，通常需要经过 2 号位附近或切换凹陷停靠区，具体由时间评分选择。
- 取完夹爪豆后直接进入左下到右上的跨场路径。

候选取货顺序示例：

```text
upper_funnel_bean -> lower_funnel_bean -> gripper_bean
lower_funnel_bean -> upper_funnel_bean -> gripper_bean
```

如果后续发现夹爪最后取导致左侧路径明显变长，可以把所有 `3!` 取货顺序都纳入枚举，并用总时间评分选择。

取货顺序评分需要包含：

- 取货停靠点切换次数。
- 相邻取货动作之间的底盘平移时间。
- 相邻取货动作之间的 yaw 旋转时间。
- 五连杆从一个取货点切换到另一个取货点的时间。
- 取完最后一种豆子后进入左下跨场入口点的时间。

## 7. 放货顺序

放货顺序从右侧目标位置和 yaw 切换代价决定。

默认候选顺序：

```text
高 y 位置 -> 中 y 位置 -> 低 y 位置
```

也就是优先生成：

```text
4/5 -> 6 -> 7/8
```

原因是主跨场路径从左下到右上进入放置区，车辆更自然地先处理上方目标，再向下扫描。

评分时重点考虑：

- 相邻放置动作之间的底盘距离。
- 相邻放置动作之间的 yaw 变化量。
- 是否能连续完成同类 yaw 的放置。
- 是否需要在右侧进行大幅掉头。
- 是否能减少漏斗和夹爪动作之间的等待。

最终放货顺序通过枚举选择。

## 8. S 型跨场与 yaw 同步

中间 S 型移动不仅是平移路径，也可以同时完成车身 yaw 旋转。

规划层应把跨场动作建模为：

```text
SMove(start_pose, end_pose)
```

其中：

- `start_pose` 包含离开取货区时的 `x, y, yaw`。
- `end_pose` 包含进入放置区时希望达到的 `x, y, yaw`。
- yaw 可以在 S 型移动过程中连续变化。
- 跨场结束 yaw 应尽量贴近第一步右侧放置动作所需 yaw。

这样评分时需要比较两类方案：

- 在 S 型通道里提前旋转，右侧到达后直接卸料。
- 保持更简单的 S 型姿态通过，进入放置区后再旋转对箱。

S 型同步旋转需要满足：

- 旋转过程中车体外廓不碰障碍柱。
- 旋转过程中夹爪、五连杆和漏斗保持安全包络。
- 货物通过障碍时高度符合规则限制。
- 旋转速度和角加速度满足底盘能力限制。

## 9. 时间评分

每个候选方案的总时间由动作调度器计算。

规划层评分公式：

```text
score = scheduled_time + risk_penalty + yaw_switch_penalty
```

其中：

- `scheduled_time`：四条资源线并行调度后的总时间。
- `risk_penalty`：靠近箱体、围栏、障碍物的风险代价。
- `yaw_switch_penalty`：右侧频繁切换车身 yaw 的代价。

第一版可以只使用：

```text
score = scheduled_time
```

等姿态库稳定后再加入风险项。

## 10. 枚举流程

规划主流程：

```text
1. 根据 drop_assignment 得到 bean -> target_drop_position。
2. 枚举 6 种 bean -> carrier 分配。
3. 为每种分配查 drop_pose_library，生成每个豆子的放置候选姿态。
4. 为每个取货位生成 pickup_station + yaw + arm 的取货候选姿态。
5. 枚举取货顺序，并枚举每个取货动作对应的 PickupPose。
6. 枚举放货顺序。
7. 生成动作块：
   - 左侧取货与装载
   - 左下到右上的 S 型跨场，并枚举跨场结束 yaw
   - 右侧停车、yaw 调整、放货
8. 调用调度器计算四条资源线并行后的总时间。
9. 选择 score 最小的方案。
```

## 11. 与 `Motions` 的关系

`Planner` 不直接计算底层运动轨迹。

`Planner` 调用 `Motions` 生成动作块：

- `move_to_pickup(pos)`
- `move_to_pickup_pose(pickup_pose)`
- `load_to_upper_funnel(bean)`
- `load_to_lower_funnel(bean)`
- `hold_by_gripper(bean)`
- `s_move_to_drop_zone(end_yaw)`
- `move_to_drop_pose(drop_pose)`
- `dump_upper_funnel()`
- `dump_lower_funnel()`
- `release_gripper()`

每个动作块提供：

- 所属资源线。
- 持续时间。
- 起止状态。
- 依赖事件。
- 可采样控制目标。

`Planner` 只关心动作块能否调度和总时间。

## 12. 当前待确认数据

规划代码落地前，需要确认这些数据：

- 4 号位四种漏斗放法的精确停车点和 yaw。
- 8 号位四种漏斗放法的精确停车点和 yaw。
- 5、6、7 号位的漏斗候选 yaw 集合。
- 夹爪取货时各取货位的可行 yaw 采样范围。
- 夹爪放货时各放置位的可行 yaw 采样范围。
- `pickup_station_12_recess` 的停车点坐标。
- `pickup_station_12_recess` 对 1 号位和 2 号位的可达 yaw 范围。
- `pickup_station_23_recess` 的停车点坐标。
- `pickup_station_23_recess` 对 2 号位和 3 号位的可达 yaw 范围。
- 1 号位与 3 号位之间切换凹陷停靠区的最短路径。
- 4 到 8 号位每个放置位的停车点坐标。
- 上漏斗在每个放置位的可行卸料姿态。
- 下漏斗在每个放置位的可行卸料姿态。
- 夹爪在每个放置位的可行释放姿态。
- 从取货区进入 S 型路径的左下入口点。
- 从 S 型路径进入放置区的右上出口点。
- S 型跨场过程中允许的 yaw 变化范围。
- 右侧扫描时的安全车身外廓。
