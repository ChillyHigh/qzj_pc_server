from __future__ import annotations

import unittest
from dataclasses import dataclass

import numpy as np

import funnel
from connection import FLAG_LOWER_FUNNEL_OPEN, FLAG_UPPER_FUNNEL_OPEN
from connection.client import MachineState
from connection.protocol import Feedback
from executor import ExecutionError, MissionExecutor
from plan import AbstractNode, ActionNode, DAG, WaitNode


class FakePath:
    def __init__(self, dim: int, duration: float, base: float = 0.0) -> None:
        self.dof = dim
        self.duration = duration
        self.calls: list[tuple[float, int]] = []
        self.base = base

    def __call__(self, path_positions, order: int = 0):
        t = float(path_positions)
        self.calls.append((t, order))
        if order == 0:
            return np.full((self.dof,), self.base + t)
        if order == 1:
            return np.full((self.dof,), 1.0)
        raise ValueError(f"unexpected order: {order}")


@dataclass(frozen=True, slots=True)
class XAtLeastTarget:
    threshold: float

    def satisfied(self, feedback: object) -> bool:
        if not isinstance(feedback, Feedback):
            return False
        return feedback.x >= self.threshold


class FakeClient:
    def __init__(self) -> None:
        self.state = MachineState()
        self.feedback: Feedback | None = None
        self.error: Exception | None = None
        self.sent: list[MachineState] = []

    def send_command(self, state: MachineState) -> None:
        self.state = state
        self.sent.append(state)


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(seconds, 0.0)


class ExecutorDAGTest(unittest.TestCase):
    def test_wait_and_action_from_same_dep_unlocks_later_action(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        start = AbstractNode("start")
        chassis_path = FakePath(3, duration=0.03, base=1.0)
        arm_path = FakePath(5, duration=0.02, base=2.0)
        chassis = ActionNode("chassis_move", deps=[start], kind="chassis", path=chassis_path)
        wait = WaitNode("wait_chassis", deps=[start], target=XAtLeastTarget(1.0), timeout=1.0)
        arm = ActionNode("arm_move", deps=[wait], kind="arm", path=arm_path)
        dag = DAG([start, chassis, wait, arm])
        executor = MissionExecutor(client, control_hz=100.0, clock=clock, sleep=clock.sleep)

        original_send = client.send_command

        def send_and_publish(state: MachineState) -> None:
            original_send(state)
            if len(client.sent) >= 2:
                client.feedback = Feedback(
                    x=1.0,
                    y=state.y,
                    yaw=state.yaw,
                    h=state.h,
                    q1=state.q1,
                    q2=state.q2,
                )

        client.send_command = send_and_publish  # type: ignore[method-assign]

        result = executor.run(dag)

        self.assertTrue(result.success)
        self.assertEqual(result.completed_nodes, 4)
        self.assertGreaterEqual(len(client.sent), 4)
        self.assertIn((0.0, 0), chassis_path.calls)
        self.assertIn((0.0, 1), chassis_path.calls)
        self.assertIn((0.0, 0), arm_path.calls)
        self.assertTrue(any(call[0] > 0.0 and call[1] == 0 for call in chassis_path.calls))

    def test_same_kind_actions_do_not_overlap(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        start = AbstractNode("start")
        first_path = FakePath(3, duration=0.02, base=1.0)
        second_path = FakePath(3, duration=0.02, base=5.0)
        first = ActionNode("first", deps=[start], kind="chassis", path=first_path)
        second = ActionNode("second", deps=[start], kind="chassis", path=second_path)
        dag = DAG([start, first, second])
        executor = MissionExecutor(client, control_hz=100.0, clock=clock, sleep=clock.sleep)

        executor.run(dag)

        self.assertEqual(second_path.calls[0], (0.0, 0))
        self.assertGreaterEqual(clock.now, first_path.duration + second_path.duration)
        self.assertAlmostEqual(client.state.x, 5.0 + second_path.duration)
        self.assertEqual(client.state.dx, 0.0)

    def test_wait_timeout_fails_explicitly(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        start = AbstractNode("start")
        wait = WaitNode("never", deps=[start], target=XAtLeastTarget(1.0), timeout=0.02)
        dag = DAG([start, wait])
        executor = MissionExecutor(client, control_hz=100.0, clock=clock, sleep=clock.sleep)

        with self.assertRaises(ExecutionError):
            executor.run(dag)

    def test_funnel_flags_path_contract(self) -> None:
        path = funnel.set(upper_open=True, lower_open=True)

        np.testing.assert_allclose(path(0.0, order=0), (FLAG_UPPER_FUNNEL_OPEN | FLAG_LOWER_FUNNEL_OPEN,))
        np.testing.assert_allclose(path(0.0, order=1), (0.0,))
        self.assertEqual(path.duration, 0.0)

    def test_funnel_flags_action_updates_state(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        start = AbstractNode("start")
        open_upper = ActionNode("open_upper", deps=[start], kind="flags", path=funnel.upper(True))
        dag = DAG([start, open_upper])
        executor = MissionExecutor(client, control_hz=100.0, clock=clock, sleep=clock.sleep)

        executor.run(dag)

        self.assertEqual(client.state.flags, FLAG_UPPER_FUNNEL_OPEN)


if __name__ == "__main__":
    unittest.main()
