from __future__ import annotations

import unittest

import numpy as np

from connection.client import MachineState
from executor import MissionExecutor
from plan import ActionNode, DAG, DelayNode, StartNode


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


class FakeClient:
    def __init__(self) -> None:
        self.feedback = None
        self.error = None
        self.sent: list[MachineState] = []

    def send_command(self, state: MachineState) -> None:
        self.sent.append(state)


class ManualClock:
    def __init__(self) -> None:
        self.now = 0.0

    def __call__(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += max(seconds, 0.0)


class DelayNodeTest(unittest.TestCase):
    def test_delay_node_unlocks_child_after_duration(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        initial_state = MachineState(x=1.0, y=2.0, yaw=3.0, h=4.0)
        start = StartNode("start", initial_state)
        delay = DelayNode("delay_1s", deps=[start], duration=1.0)
        path = FakePath(5, duration=0.02, base=2.0)
        arm = ActionNode("arm_after_delay", deps=[delay], kind="arm", path=path)
        dag = DAG([start, delay, arm])
        executor = MissionExecutor(client, control_hz=100.0, clock=clock, sleep=clock.sleep)

        result = executor.run(dag)

        self.assertTrue(result.success)
        self.assertEqual(result.completed_nodes, 3)
        self.assertGreaterEqual(clock.now, 1.0 + path.duration)
        self.assertIn((0.0, 0), path.calls)
        self.assertEqual(client.sent[0], initial_state)

    def test_delay_node_rejects_negative_duration(self) -> None:
        start = StartNode("start", MachineState())
        delay = DelayNode("bad_delay", deps=[start], duration=-1.0)

        with self.assertRaises(ValueError):
            DAG([start, delay])

    def test_dag_requires_exactly_one_start_node(self) -> None:
        with self.assertRaises(ValueError):
            DAG([DelayNode("delay", duration=0.0)])

        with self.assertRaises(ValueError):
            DAG(
                [
                    StartNode("start_1", MachineState()),
                    StartNode("start_2", MachineState()),
                ]
            )

    def test_start_node_rejects_non_machine_state(self) -> None:
        start = StartNode("start", object())  # type: ignore[arg-type]

        with self.assertRaises(ValueError):
            DAG([start])


if __name__ == "__main__":
    unittest.main()
