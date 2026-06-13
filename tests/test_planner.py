from __future__ import annotations

import math
import unittest

import arm
from connection.protocol import Feedback
from connection.client import MachineState
from plan import ActionNode, WaitNode
from planner import ChassisReachedTarget, Planner

INIT_CHASSIS = (0.3, 0.0, 0.0)
INIT_ARM = (0.055, 0.0, 0.0, 0.3, 0.0)
INIT_Q1, INIT_Q2, INIT_GRIPPER_YAW = arm.FiveBarKinematics().ik(INIT_ARM[0], INIT_ARM[1], INIT_ARM[2])
INIT_STATE = MachineState(
    x=INIT_CHASSIS[0],
    y=INIT_CHASSIS[1],
    yaw=INIT_CHASSIS[2],
    h=INIT_ARM[3],
    q1=INIT_Q1,
    q2=INIT_Q2,
    gripper_yaw=INIT_GRIPPER_YAW,
    gripper_opening=INIT_ARM[4],
)


class TestChassisReachedTarget(unittest.TestCase):
    def test_satisfied_when_within_tolerance(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0), pos_tol=0.05, yaw_tol=0.1)
        fb = Feedback(x=1.02, y=2.01, yaw=0.03, h=0.3, q1=3.6, q2=3.6)
        self.assertTrue(target.satisfied(fb))

    def test_not_satisfied_when_position_far(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0), pos_tol=0.05, yaw_tol=0.1)
        fb = Feedback(x=2.0, y=3.0, yaw=0.0, h=0.3, q1=3.6, q2=3.6)
        self.assertFalse(target.satisfied(fb))

    def test_not_satisfied_when_yaw_far(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0), pos_tol=0.05, yaw_tol=0.1)
        fb = Feedback(x=1.0, y=2.0, yaw=0.5, h=0.3, q1=3.6, q2=3.6)
        self.assertFalse(target.satisfied(fb))

    def test_yaw_wraparound(self) -> None:
        target = ChassisReachedTarget((0.0, 0.0, math.radians(350.0)), pos_tol=0.1, yaw_tol=math.radians(20.0))
        fb = Feedback(x=0.0, y=0.0, yaw=math.radians(10.0), h=0.3, q1=3.6, q2=3.6)
        self.assertTrue(target.satisfied(fb))

    def test_not_satisfied_with_none_feedback(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0))
        self.assertFalse(target.satisfied(None))

    def test_not_satisfied_with_wrong_type(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0))
        self.assertFalse(target.satisfied("not feedback"))

    def test_not_satisfied_with_object_missing_attrs(self) -> None:
        target = ChassisReachedTarget((1.0, 2.0, 0.0))

        class Bad:
            pass

        self.assertFalse(target.satisfied(Bad()))


class TestPlannerGeneration(unittest.TestCase):
    def setUp(self) -> None:
        self.planner = Planner(INIT_STATE)

    def test_generates_valid_dag(self) -> None:
        dag, estimated_time = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        self.assertGreater(len(dag.nodes), 0)
        self.assertGreater(estimated_time, 0.0)

    def test_node_names_are_unique(self) -> None:
        dag, _ = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        names = [n.name for n in dag.nodes]
        self.assertEqual(len(names), len(set(names)))

    def test_all_beans_have_pickup_and_drop_in_sequence(self) -> None:
        """取货顺序固定 1→3→2，跨场只用一次，放货覆盖 3 个 bean。"""
        dag, _ = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        names = [n.name for n in dag.nodes]

        pick_1_idx = next(i for i, n in enumerate(names) if "pick_1" in n)
        pick_3_idx = next(i for i, n in enumerate(names) if "pick_3" in n)
        pick_2_idx = next(i for i, n in enumerate(names) if "pick_2" in n)
        self.assertLess(pick_1_idx, pick_3_idx)
        self.assertLess(pick_3_idx, pick_2_idx)

        self.assertTrue(any("s_cross" in n for n in names))

        drop_count = sum(1 for n in names if "drop_" in n and "_done" in n)
        self.assertEqual(drop_count, 3)

    def test_s_cross_precondition_check(self) -> None:
        """取货 2 号位后在左下，满足 s_cross 前置条件。"""
        dag, _ = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        cross_nodes = [n for n in dag.nodes if isinstance(n, ActionNode) and n.kind == "chassis" and "s_cross" in n.name]
        self.assertEqual(len(cross_nodes), 1)

    def test_all_action_nodes_have_paths(self) -> None:
        dag, _ = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        for node in dag.nodes:
            if isinstance(node, ActionNode):
                self.assertIsNotNone(node.path, f"{node.name} path 为 None")

    def test_different_assignments_produce_dag(self) -> None:
        dag1, time1 = self.planner.generate([1, 2, 3], [1, 2, 3, 4, 5])
        dag2, time2 = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        self.assertGreater(len(dag1.nodes), 0)
        self.assertGreater(len(dag2.nodes), 0)
        self.assertGreater(time1, 0.0)
        self.assertGreater(time2, 0.0)

    def test_dag_is_acyclic(self) -> None:
        """DAG.__post_init__ 已经校验无环，不抛异常即通过。"""
        dag, estimated_time = self.planner.generate([3, 1, 2], [4, 1, 2, 3, 5])
        self.assertGreater(len(dag.nodes), 0)
        self.assertGreater(estimated_time, 0.0)

    def test_gripper_only_assignment_works(self) -> None:
        """所有 bean 都分配到 drop 位 4/5/6，planner 能从候选选择最优 DAG。"""
        planner = Planner(INIT_STATE)
        dag, estimated_time = planner.generate([1, 2, 3], [1, 2, 3, 4, 5])
        self.assertGreater(len(dag.nodes), 0)
        self.assertGreater(estimated_time, 0.0)
        for node in dag.nodes:
            if isinstance(node, ActionNode):
                self.assertIsNotNone(node.path, f"{node.name} path 为 None")


class TestPlannerEdgeCases(unittest.TestCase):
    def setUp(self) -> None:
        self.planner = Planner(INIT_STATE)

    def test_invalid_pickup_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            self.planner.generate([1, 2], [1, 2, 3, 4, 5])

    def test_invalid_drop_length_raises(self) -> None:
        with self.assertRaises(ValueError):
            self.planner.generate([1, 2, 3], [1, 2, 3, 4])

    def test_missing_bean_in_pickup_raises(self) -> None:
        with self.assertRaises(ValueError):
            self.planner.generate([1, 2, 4], [1, 2, 3, 4, 5])

    def test_missing_bean_in_drop_raises(self) -> None:
        with self.assertRaises(ValueError):
            self.planner.generate([1, 2, 3], [4, 5, 6, 7, 8])


if __name__ == "__main__":
    unittest.main()
