from __future__ import annotations

import math
from collections import deque
from typing import TYPE_CHECKING

import arm
import chassis
import funnel
from plan import AbstractNode, ActionNode, DAG, WaitNode, DelayNode

from . import config

if TYPE_CHECKING:
    from connection.protocol import Feedback

DropCarrier = config.DropCarrier
DrivePose = tuple[float, float, float]
ArmCartesian = tuple[float, float, float, float, float]
BeanId = int
PosId = int


def _path_end_drive(path: object) -> DrivePose:
    import numpy as np

    duration = float(getattr(path, "duration"))
    q = np.asarray(path(duration, order=0), dtype=float).reshape(-1)
    if q.shape != (3,):
        raise RuntimeError("chassis path 终点维度错误。")
    return (float(q[0]), float(q[1]), float(q[2]))


def _angle_diff(a: float, b: float) -> float:
    d = (a - b) % (2.0 * math.pi)
    if d > math.pi:
        d -= 2.0 * math.pi
    return d


def _shortest_yaw_delta(from_yaw: float, to_yaw: float) -> float:
    delta = (to_yaw - from_yaw + math.pi) % (2.0 * math.pi) - math.pi
    if math.isclose(delta, -math.pi, abs_tol=1e-9):
        return math.pi
    return delta


def _clamp_joint_state(
    state: tuple[float, ...],
    q_min: tuple[float, ...],
    q_max: tuple[float, ...],
) -> tuple[float, ...]:
    return tuple(max(low, min(high, float(v))) for v, low, high in zip(state, q_min, q_max))


class ChassisReachedTarget:
    """WaitNode target：底盘 (x, y, yaw) 到达目标容差内时 satisfied。"""

    def __init__(
        self,
        target: tuple[float, float, float],
        pos_tol: float = config.WAIT_POS_TOLERANCE,
        yaw_tol: float = config.WAIT_YAW_TOLERANCE,
    ) -> None:
        self._target_x = target[0]
        self._target_y = target[1]
        self._target_yaw = target[2]
        self._pos_tol = pos_tol
        self._yaw_tol = yaw_tol

    def satisfied(self, feedback: object) -> bool:
        if feedback is None:
            return False
        try:
            fx = float(getattr(feedback, "x"))
            fy = float(getattr(feedback, "y"))
            fyaw = float(getattr(feedback, "yaw"))
        except (AttributeError, TypeError):
            return False
        dx = fx - self._target_x
        dy = fy - self._target_y
        dyaw = abs(_angle_diff(fyaw, self._target_yaw))
        return math.hypot(dx, dy) <= self._pos_tol and dyaw <= self._yaw_tol


class Planner:
    """解析抽签并生成比赛 DAG。"""

    def __init__(
        self,
        init_chassis: tuple[float, float, float],
        init_arm: tuple[float, float, float, float, float],  # x, y, g_yaw, height, open
    ) -> None:
        self._initial_drive: DrivePose = init_chassis
        self._initial_arm: ArmCartesian = init_arm
        self._drive: DrivePose = self._initial_drive
        self._arm: ArmCartesian = self._initial_arm
        self._counter = 0

    def _next_drive(self, target: DrivePose) -> DrivePose:
        """将 target 的 yaw 相对当前 self._drive 解缠绕，保持 yaw 连续。

        config 中的 yaw 可能在 [-π, π]，而 trajectory 输出的是解缠绕后的连续 yaw。
        这里确保 planner 追踪的底盘状态与实际 trajectory 终点一致。
        """
        dyaw = _shortest_yaw_delta(self._drive[2], target[2])
        return (target[0], target[1], self._drive[2] + dyaw)

    # ---- public API ----------------------------------------------------------

    def generate(
        self,
        pickup_assignment: list[int],
        drop_assignment: list[int],
    ) -> tuple[DAG, float]:
        if len(pickup_assignment) != 3:
            raise ValueError(f"pickup_assignment 长度必须为 3，实际为 {len(pickup_assignment)}")
        if len(drop_assignment) != 5:
            raise ValueError(f"drop_assignment 长度必须为 5，实际为 {len(drop_assignment)}")
        if set(pickup_assignment) != {1, 2, 3}:
            raise ValueError(f"pickup_assignment 必须包含 1, 2, 3，实际为 {pickup_assignment}")

        # 解析 bean → 取货位 / 放置位
        bean_pickup_pos: dict[BeanId, PosId] = {}
        for idx, bean in enumerate(pickup_assignment):
            bean_pickup_pos[bean] = idx + 1

        bean_drop_pos: dict[BeanId, PosId] = {}
        for idx, box_label in enumerate(drop_assignment):
            if box_label in (1, 2, 3):
                bean_drop_pos[box_label] = idx + 4

        missing = {1, 2, 3} - set(bean_drop_pos.keys())
        if missing:
            raise ValueError(f"drop_assignment 缺少货物 {sorted(missing)} 的目标放置位")

        # 找出取货位 2 上的 bean（固定走 gripper）
        bean_at_pos = {v: k for k, v in bean_pickup_pos.items()}
        gripper_bean = bean_at_pos[2]
        funnel_beans = [b for b in (1, 2, 3) if b != gripper_bean]

        # 只枚举 2 种：funnel_beans[0]→upper/funnel_beans[1]→lower，或反过来
        best_dag: DAG | None = None
        best_time = float("inf")

        for swap in (False, True):
            assignment: dict[BeanId, DropCarrier] = {gripper_bean: "gripper"}
            a, b = funnel_beans
            if swap:
                assignment[a] = "lower_funnel"
                assignment[b] = "upper_funnel"
            else:
                assignment[a] = "upper_funnel"
                assignment[b] = "lower_funnel"

            dag = self._build_dag(assignment, bean_pickup_pos, bean_drop_pos)
            est = self._estimate_time(dag)
            if est < best_time:
                best_time = est
                best_dag = dag

        if best_dag is None:
            raise RuntimeError("未能生成任何有效 DAG")
        return best_dag, best_time

    # ---- DAG 构建 ------------------------------------------------------------

    def _build_dag(
        self,
        assignment: dict[BeanId, DropCarrier],
        bean_pickup_pos: dict[BeanId, PosId],
        bean_drop_pos: dict[BeanId, PosId],
    ) -> DAG:
        self._counter = 0
        self._drive = self._initial_drive
        self._arm = self._initial_arm

        nodes: list[AbstractNode] = []
        start = AbstractNode(self._next_name("start"))
        nodes.append(start)

        prev = self._add_move_to_pickup_area(nodes, start)

        bean_at_pos = {v: k for k, v in bean_pickup_pos.items()}
        pickup_order = [1, 3, 2]

        arm_prev = prev
        chassis_prev = prev
        for pos in pickup_order:
            bean = bean_at_pos[pos]
            carrier = assignment[bean]
            chassis_prev, arm_prev = self._add_pickup(nodes, chassis_prev, arm_prev, pos, carrier)
        prev = chassis_prev

        # 放货顺序 + s_cross 直达首个放货位
        drop_order = self._compute_drop_order(assignment, bean_drop_pos)
        first_bean = drop_order[0]
        first_drop_pos = bean_drop_pos[first_bean]
        first_carrier = assignment[first_bean]
        first_drop_pose = config.DROP_POSES[first_drop_pos][first_carrier]
        first_drive: DrivePose = (
            first_drop_pose[config.DRIVE_X],
            first_drop_pose[config.DRIVE_Y],
            first_drop_pose[config.DRIVE_YAW],
        )

        cross_done = self._add_cross_field(nodes, prev, first_drive)

        # 首个放豆：底盘已由 s_cross 送到，跳过 chassis move
        prev = self._add_drop_first(nodes, cross_done, first_drop_pos, first_carrier)
        for bean in drop_order[1:]:
            pos = bean_drop_pos[bean]
            carrier = assignment[bean]
            prev = self._add_drop(nodes, prev, pos, carrier)

        self._add_finish(nodes, prev)

        return DAG(nodes)

    # ---- Phase 0: 移动到取货区 -----------------------------------------------

    def _add_move_to_pickup_area(
        self, nodes: list[AbstractNode], dep: AbstractNode
    ) -> AbstractNode:
        pickup_1 = config.PICKUP_POSES[1]
        raw_target: DrivePose = (
            pickup_1[config.DRIVE_X],
            pickup_1[config.DRIVE_Y],
            pickup_1[config.DRIVE_YAW],
        )
        path = chassis.direct(self._drive, raw_target)
        reached = _path_end_drive(path)
        chassis_move = ActionNode(
            name=self._next_name("chassis_to_pickup_area"),
            deps=[dep],
            kind="chassis",
            path=path,
        )
        wait = WaitNode(
            name=self._next_name("wait_pickup_area"),
            deps=[dep],
            target=ChassisReachedTarget(reached),
            timeout=config.WAIT_TIMEOUT,
        )
        nodes.extend([chassis_move, wait])
        self._drive = reached

        merge = AbstractNode(
            self._next_name("pickup_area_reached"),
            deps=[chassis_move, wait],
        )
        nodes.append(merge)
        return merge

    # ---- Phase 1: 取货 -------------------------------------------------------

    def _add_pickup(
        self,
        nodes: list[AbstractNode],
        chassis_prev: AbstractNode,
        arm_prev: AbstractNode,
        pickup_pos: PosId,
        carrier: DropCarrier,
    ) -> tuple[AbstractNode, AbstractNode]:
        # 停车点
        pose = config.PICKUP_POSES[pickup_pos]
        raw_drive_target: DrivePose = (
            pose[config.DRIVE_X],
            pose[config.DRIVE_Y],
            pose[config.DRIVE_YAW],
        )
        arm_pick_target: ArmCartesian = (
            pose[config.ARM_X],
            pose[config.ARM_Y],
            pose[config.GRIPPER_YAW],
            pose[config.H] + config.货箱高,
            config.GRIPPER_OPEN_ANGLE,
        )

        path = chassis.direct(self._drive, raw_drive_target)
        reached = _path_end_drive(path)
        chassis_move = ActionNode(
            name=self._next_name(f"chassis_to_pick_{pickup_pos}"),
            deps=[chassis_prev],
            kind="chassis",
            path=path,
        )

        nodes.extend([chassis_move])
        self._drive = reached

        arm_prep = ActionNode(
            name=self._next_name(f"arm_prep_pick_{pickup_pos}"),
            deps=[arm_prev],
            kind="arm",
            path=arm.move(
                self._arm,
                arm_pick_target,
            ),
        )

        # TODO: 调整prepare最后才打开gripper
        arm_do = ActionNode(
            name=self._next_name(f"arm_do_pick_{pickup_pos}"),
            deps=[arm_prep],
            kind="arm",
            path=arm.do_pick(
                arm_pick_target,
                bean_h=pose[config.H] + config.豆子厚度,
            ),
        )
        nodes.append(arm_prep)
        nodes.append(arm_do)

        self._arm = (arm_pick_target[0],
                     arm_pick_target[1],
                     arm_pick_target[2],
                     arm_pick_target[3],
                     config.GRIPPER_CLOSED_ANGLE
                     )

        if carrier == "gripper":

            with_draw_pos: ArmCartesian = (0.1, 0, 0, config.放漏斗高度,
                     config.GRIPPER_CLOSED_ANGLE)

            arm_withdraw = ActionNode(
                name=self._next_name(f"withdraw_arm_{pickup_pos}"),
                deps=[arm_prep],
                kind="arm",
                path=arm.move(
                    self._arm,
                    with_draw_pos
                ),
            )
            nodes.append(arm_withdraw)
            self._arm = with_draw_pos

            merge = AbstractNode(
                self._next_name(f"pick_{pickup_pos}_done"),
                deps=[chassis_move, arm_withdraw],
            )
            nodes.append(merge)

            return merge, merge
        store = self._add_funnel_load(nodes, arm_do, carrier, pickup_pos)
        return arm_do, store

    def _add_funnel_load(
        self,
        nodes: list[AbstractNode],
        arm_after_pick: ActionNode,
        funnel_kind: DropCarrier,
        pickup_pos: PosId,
    ) -> AbstractNode:

        funnel_xy = config.FUNNEL_ARM_TARGET[funnel_kind]
        funnel_target: ArmCartesian = (
            funnel_xy[0],
            funnel_xy[1],
            0.0,
            config.放漏斗高度,
            config.GRIPPER_CLOSED_ANGLE,
        )

        arm_to_funnel = ActionNode(
            name=self._next_name(f"arm_to_{funnel_kind}_p{pickup_pos}"),
            deps=[arm_after_pick],
            kind="arm",
            path=arm.move(self._arm, funnel_target),
        )
        nodes.append(arm_to_funnel)

        current_arm: ArmCartesian = (
            funnel_target[0],
            funnel_target[1],
            funnel_target[2],
            funnel_target[3],
            config.GRIPPER_CLOSED_ANGLE,
        )

        arm_release = ActionNode(
            name=self._next_name(f"arm_release_{funnel_kind}_p{pickup_pos}"),
            deps=[arm_to_funnel],
            kind="arm",
            path=arm.set_gripper(current_arm, config.GRIPPER_OPEN_ANGLE),
        )
        nodes.append(arm_release)

        delay_1s = DelayNode(
            name = self._next_name("delay_1s"),
            deps=[arm_release],
            duration=1.0)
        nodes.append(delay_1s)

        current_arm: ArmCartesian = (
            funnel_target[0],
            funnel_target[1],
            funnel_target[2],
            funnel_target[3],
            config.GRIPPER_OPEN_ANGLE,
        )
        arm_close = ActionNode(
            name=self._next_name(f"arm_close_{funnel_kind}_p{pickup_pos}"),
            deps=[delay_1s],
            kind="arm",
            path=arm.set_gripper(current_arm, config.GRIPPER_CLOSED_ANGLE),
        )
        nodes.append(arm_close)

        self._arm = (
            funnel_target[0],
            funnel_target[1],
            funnel_target[2],
            funnel_target[3],
            config.GRIPPER_CLOSED_ANGLE,
        )

        return arm_close

    # ---- Phase 2: 跨场 -------------------------------------------------------

    def _add_cross_field(
        self, nodes: list[AbstractNode], prev: AbstractNode, target: DrivePose,
    ) -> AbstractNode:
        if not (self._drive[0] < -1.0 and self._drive[1] < 0.0):
            raise ValueError(
                f"s_cross 要求 start.x < -1.0 且 start.y < 0.0，"
                f"当前底盘为 ({self._drive[0]:.3f}, {self._drive[1]:.3f})"
            )

        path = chassis.s_cross(self._drive, target)
        reached = _path_end_drive(path)
        chassis_cross = ActionNode(
            name=self._next_name("chassis_s_cross"),
            deps=[prev],
            kind="chassis",
            path=path,
        )
        wait_cross = WaitNode(
            name=self._next_name("wait_cross_done"),
            deps=[prev],
            target=ChassisReachedTarget(reached),
            timeout=config.WAIT_CROSS_TIMEOUT,
        )
        nodes.extend([chassis_cross, wait_cross])
        self._drive = reached

        merge = AbstractNode(
            self._next_name("cross_done"),
            deps=[chassis_cross, wait_cross],
        )
        nodes.append(merge)
        return merge

    # ---- Phase 3: 放货 -------------------------------------------------------

    def _compute_drop_order(
        self,
        assignment: dict[BeanId, DropCarrier],
        bean_drop_pos: dict[BeanId, PosId],
    ) -> list[BeanId]:
        """按箱号 4→5→6→7→8 顺序放豆，从上到下。"""
        return sorted(bean_drop_pos.keys(), key=lambda b: bean_drop_pos[b])

    def _add_drop_first(
        self,
        nodes: list[AbstractNode],
        prev: AbstractNode,
        drop_pos: PosId,
        carrier: DropCarrier,
    ) -> AbstractNode:
        """首个放豆：s_cross 已送到，跳过底盘移动。"""
        pose = config.DROP_POSES[drop_pos][carrier]
        raw_drive_target: DrivePose = (
            pose[config.DRIVE_X],
            pose[config.DRIVE_Y],
            pose[config.DRIVE_YAW],
        )
        if not (
            math.isclose(self._drive[0], raw_drive_target[0], abs_tol=1e-6)
            and math.isclose(self._drive[1], raw_drive_target[1], abs_tol=1e-6)
            and math.isclose(_angle_diff(self._drive[2], raw_drive_target[2]), 0.0, abs_tol=1e-6)
        ):
            raise RuntimeError("首个放货位与 s_cross 终点不一致。")

        if carrier in ("upper_funnel", "lower_funnel"):
            open_func = funnel.upper if carrier == "upper_funnel" else funnel.lower
            open_node = ActionNode(
                name=self._next_name(f"open_{carrier}_drop_{drop_pos}"),
                deps=[prev],
                kind="flags",
                path=open_func(True),
            )
            close_node = ActionNode(
                name=self._next_name(f"close_{carrier}_drop_{drop_pos}"),
                deps=[open_node],
                kind="flags",
                path=open_func(False),
            )
            nodes.extend([open_node, close_node])

            merge = AbstractNode(
                self._next_name(f"drop_{drop_pos}_{carrier}_done"),
                deps=[prev, close_node],
            )
            nodes.append(merge)
            return merge

        arm_drop_target: ArmCartesian = (
            pose[config.ARM_X],
            pose[config.ARM_Y],
            pose[config.GRIPPER_YAW],
            pose[config.H],
            config.GRIPPER_CLOSED_ANGLE,
        )
        arm_to_drop = ActionNode(
            name=self._next_name(f"arm_to_drop_{drop_pos}"),
            deps=[prev],
            kind="arm",
            path=arm.move(self._arm, arm_drop_target),
        )
        nodes.append(arm_to_drop)

        arm_release = ActionNode(
            name=self._next_name(f"arm_release_drop_{drop_pos}"),
            deps=[arm_to_drop],
            kind="arm",
            path=arm.set_gripper(
                (
                    arm_drop_target[0],
                    arm_drop_target[1],
                    arm_drop_target[2],
                    arm_drop_target[3],
                    arm_drop_target[4],
                ),
                config.GRIPPER_OPEN_ANGLE,
            ),
        )
        nodes.append(arm_release)

        self._arm = (
            arm_drop_target[0],
            arm_drop_target[1],
            arm_drop_target[2],
            arm_drop_target[3],
            config.GRIPPER_OPEN_ANGLE,
        )

        merge = AbstractNode(
            self._next_name(f"drop_{drop_pos}_gripper_done"),
            deps=[prev, arm_release],
        )
        nodes.append(merge)
        return merge

    def _add_drop(
        self,
        nodes: list[AbstractNode],
        prev: AbstractNode,
        drop_pos: PosId,
        carrier: DropCarrier,
    ) -> AbstractNode:
        pose = config.DROP_POSES[drop_pos][carrier]
        raw_drive_target: DrivePose = (
            pose[config.DRIVE_X],
            pose[config.DRIVE_Y],
            pose[config.DRIVE_YAW],
        )
        path = chassis.direct(self._drive, raw_drive_target)
        reached = _path_end_drive(path)
        chassis_move = ActionNode(
            name=self._next_name(f"chassis_to_drop_{drop_pos}_{carrier}"),
            deps=[prev],
            kind="chassis",
            path=path,
        )
        wait = WaitNode(
            name=self._next_name(f"wait_drop_{drop_pos}_{carrier}"),
            deps=[prev],
            target=ChassisReachedTarget(reached),
            timeout=config.WAIT_TIMEOUT,
        )
        nodes.extend([chassis_move, wait])
        self._drive = reached

        if carrier in ("upper_funnel", "lower_funnel"):
            open_func = funnel.upper if carrier == "upper_funnel" else funnel.lower
            open_node = ActionNode(
                name=self._next_name(f"open_{carrier}_drop_{drop_pos}"),
                deps=[wait],
                kind="flags",
                path=open_func(True),
            )
            close_node = ActionNode(
                name=self._next_name(f"close_{carrier}_drop_{drop_pos}"),
                deps=[open_node],
                kind="flags",
                path=open_func(False),
            )
            nodes.extend([open_node, close_node])

            merge = AbstractNode(
                self._next_name(f"drop_{drop_pos}_{carrier}_done"),
                deps=[chassis_move, close_node],
            )
            nodes.append(merge)
            return merge

        arm_drop_target: ArmCartesian = (
            pose[config.ARM_X],
            pose[config.ARM_Y],
            pose[config.GRIPPER_YAW],
            pose[config.H],
            config.GRIPPER_CLOSED_ANGLE,
        )
        arm_to_drop = ActionNode(
            name=self._next_name(f"arm_to_drop_{drop_pos}"),
            deps=[wait],
            kind="arm",
            path=arm.move(self._arm, arm_drop_target),
        )
        nodes.append(arm_to_drop)

        arm_release = ActionNode(
            name=self._next_name(f"arm_release_drop_{drop_pos}"),
            deps=[arm_to_drop],
            kind="arm",
            path=arm.set_gripper(
                (
                    arm_drop_target[0],
                    arm_drop_target[1],
                    arm_drop_target[2],
                    arm_drop_target[3],
                    arm_drop_target[4],
                ),
                config.GRIPPER_OPEN_ANGLE,
            ),
        )
        nodes.append(arm_release)

        self._arm = (
            arm_drop_target[0],
            arm_drop_target[1],
            arm_drop_target[2],
            arm_drop_target[3],
            config.GRIPPER_OPEN_ANGLE,
        )

        merge = AbstractNode(
            self._next_name(f"drop_{drop_pos}_gripper_done"),
            deps=[chassis_move, arm_release],
        )
        nodes.append(merge)
        return merge

    # ---- Phase 4: 结束 -------------------------------------------------------

    def _add_finish(self, nodes: list[AbstractNode], prev: AbstractNode) -> None:
        path = chassis.direct(self._drive, config.FINISH_DRIVE)
        reached = _path_end_drive(path)
        chassis_finish = ActionNode(
            name=self._next_name("chassis_to_finish"),
            deps=[prev],
            kind="chassis",
            path=path,
        )
        wait_finish = WaitNode(
            name=self._next_name("wait_finish"),
            deps=[prev],
            target=ChassisReachedTarget(reached),
            timeout=config.WAIT_TIMEOUT,
        )
        finish_arrived = AbstractNode(
            self._next_name("finish_arrived"),
            deps=[chassis_finish, wait_finish],
        )
        close_all = ActionNode(
            name=self._next_name("close_all_funnels"),
            deps=[finish_arrived],
            kind="flags",
            path=funnel.close_all(),
        )
        final = AbstractNode(
            self._next_name("mission_complete"),
            deps=[close_all],
        )
        nodes.extend([chassis_finish, wait_finish, finish_arrived, close_all, final])

    # ---- 时间估算 ------------------------------------------------------------

    def _estimate_time(self, dag: DAG) -> float:
        node_end: dict[str, float] = {}
        kind_end: dict[str, float] = {"chassis": 0.0, "arm": 0.0, "flags": 0.0}

        order = self._topological_order(dag.nodes)
        for node in order:
            dep_end = max((node_end[d.name] for d in node.deps), default=0.0)

            if isinstance(node, ActionNode):
                start = max(dep_end, kind_end[node.kind])
                dur = float(node.path.duration) if node.path is not None else 0.0
                node_end[node.name] = start + dur
                kind_end[node.kind] = start + dur
            elif isinstance(node, WaitNode):
                node_end[node.name] = dep_end + 0.1
            elif isinstance(node, DelayNode):
                node_end[node.name] = dep_end + float(node.duration)
            else:
                node_end[node.name] = dep_end

        return max(node_end.values(), default=0.0)

    @staticmethod
    def _topological_order(nodes: list[AbstractNode]) -> list[AbstractNode]:
        from plan import children as plan_children
        from plan import dep_left

        remaining = dep_left(nodes)
        queue: deque[AbstractNode] = deque(n for n in nodes if remaining[n] == 0)
        result: list[AbstractNode] = []
        child_map = plan_children(nodes)
        while queue:
            node = queue.popleft()
            result.append(node)
            for child in child_map[node]:
                remaining[child] -= 1
                if remaining[child] == 0:
                    queue.append(child)
        return result

    # ---- helpers -------------------------------------------------------------

    def _next_name(self, base: str) -> str:
        self._counter += 1
        return f"{base}_{self._counter}"
