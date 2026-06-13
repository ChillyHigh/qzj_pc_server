from __future__ import annotations

import os
import tempfile
import unittest
from pathlib import Path

import numpy as np

from debug import draw_dag
from connection.client import MachineState
from plan import ActionNode, DAG, StartNode, WaitNode


os.environ.setdefault("MPLCONFIGDIR", "/private/tmp/mplconfig")


class FakePath:
    def __init__(self, dim: int, duration: float) -> None:
        self.dim = dim
        self.duration = duration

    def __call__(self, path_positions, order: int = 0):
        if order in (0, 1):
            return np.zeros((self.dim,), dtype=float)
        raise ValueError(f"unexpected order: {order}")


class AlwaysTarget:
    def satisfied(self, feedback: object) -> bool:
        return True


class DebugDAGPlotTest(unittest.TestCase):
    def test_draw_dag_writes_png(self) -> None:
        start = StartNode("start", MachineState())
        chassis = ActionNode("go", deps=[start], kind="chassis", path=FakePath(3, 1.2))
        wait = WaitNode("feedback", deps=[chassis], target=AlwaysTarget(), timeout=2.0)
        arm = ActionNode("pick", deps=[wait], kind="arm", path=FakePath(5, 0.8))
        dag = DAG([start, chassis, wait, arm])

        with tempfile.TemporaryDirectory() as tmp_dir:
            output = draw_dag(dag, Path(tmp_dir) / "dag.png")

            self.assertTrue(output.exists())
            self.assertGreater(output.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
