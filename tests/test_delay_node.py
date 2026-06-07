from __future__ import annotations

import unittest

import numpy as np

from connection.client import MachineState
from executor import MissionExecutor
from plan import AbstractNode, ActionNode, DAG, DelayNode


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
        self.state = MachineState()
        self.feedback = None
        self.error = None
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


class DelayNodeTest(unittest.TestCase):
    def test_delay_node_unlocks_child_after_duration(self) -> None:
        client = FakeClient()
        clock = ManualClock()
        start = AbstractNode("start")
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

    def test_delay_node_rejects_negative_duration(self) -> None:
        start = AbstractNode("start")
        delay = DelayNode("bad_delay", deps=[start], duration=-1.0)

        with self.assertRaises(ValueError):
            DAG([start, delay])


if __name__ == "__main__":
    unittest.main()
