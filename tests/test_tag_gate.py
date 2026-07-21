from __future__ import annotations

import math
import socket
import tempfile
from pathlib import Path

import pytest

import config
from connection.client import MachineState
from executor.tag_gate import GATE_CLOSED, GATE_OPEN, TagGateError, TagGateSender, is_slow


def test_speed_limits_are_strict() -> None:
    assert is_slow(MachineState(dx=0.01, dy=0.02, dyaw=math.radians(1.0)))
    assert not is_slow(MachineState(dx=config.TAG_GATE_LINEAR_LIMIT))
    assert not is_slow(MachineState(dyaw=config.TAG_GATE_ANGULAR_LIMIT))
    assert not is_slow(MachineState(dx=0.04, dy=0.04))


def test_sender_sends_changes_and_heartbeat(monkeypatch) -> None:
    with tempfile.TemporaryDirectory(prefix="tag_gate_", dir="/tmp") as temp_dir:
        uds_path = str(Path(temp_dir) / "gate.sock")
        monkeypatch.setattr(config, "TAG_GATE_UDS_PATH", uds_path)
        receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        receiver.bind(uds_path)
        receiver.setblocking(False)
        sender = TagGateSender()

        try:
            sender.publish(MachineState(), 0.0)
            assert receiver.recv(1) == GATE_OPEN

            sender.publish(MachineState(), config.TAG_GATE_HEARTBEAT_S / 2.0)
            with pytest.raises(BlockingIOError):
                receiver.recv(1)

            sender.publish(MachineState(), config.TAG_GATE_HEARTBEAT_S)
            assert receiver.recv(1) == GATE_OPEN

            sender.publish(MachineState(dx=config.TAG_GATE_LINEAR_LIMIT), 0.11)
            assert receiver.recv(1) == GATE_CLOSED
        finally:
            receiver.close()
            Path(uds_path).unlink()


def test_sender_fails_without_receiver(monkeypatch) -> None:
    with tempfile.TemporaryDirectory(prefix="tag_gate_", dir="/tmp") as temp_dir:
        uds_path = str(Path(temp_dir) / "missing.sock")
        monkeypatch.setattr(config, "TAG_GATE_UDS_PATH", uds_path)
        sender = TagGateSender()

        with pytest.raises(TagGateError, match="UDS 发送失败"):
            sender.publish(MachineState(), 0.0)
