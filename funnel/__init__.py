from __future__ import annotations

import numpy as np

from connection import FLAG_LOWER_FUNNEL_OPEN, FLAG_UPPER_FUNNEL_OPEN

from . import config


class FunnelPathError(ValueError):
    """漏斗 flags path 生成失败。"""


class _FlagsPath:
    def __init__(self, flags: int, duration: float) -> None:
        if not 0 <= flags <= FLAG_UPPER_FUNNEL_OPEN | FLAG_LOWER_FUNNEL_OPEN:
            raise FunnelPathError(f"flags 只能包含上下漏斗开关位：{flags}")
        self.flags = int(flags)
        self.duration = float(duration)
        if self.duration < 0.0:
            raise FunnelPathError(f"flags path duration 不能小于 0：{self.duration}")

    def __call__(self, path_positions, order: int = 0):
        if order == 0:
            value = float(self.flags)
        elif order == 1:
            value = 0.0
        else:
            raise FunnelPathError(f"flags path 不支持 order={order}")
        arr = np.asarray(path_positions)
        if arr.shape == ():
            return np.asarray((value,), dtype=float)
        return np.repeat(np.asarray([[value]], dtype=float), arr.size, axis=0)


def set(upper_open: bool, lower_open: bool) -> _FlagsPath:
    flags = 0
    if upper_open:
        flags |= FLAG_UPPER_FUNNEL_OPEN
    if lower_open:
        flags |= FLAG_LOWER_FUNNEL_OPEN
    duration = config.OPEN_DURATION if flags != 0 else 0.0
    return _FlagsPath(flags, duration)


def upper(open: bool) -> _FlagsPath:
    return set(open, False)


def lower(open: bool) -> _FlagsPath:
    return set(False, open)


def close_all() -> _FlagsPath:
    return set(False, False)


__all__ = [
    "FunnelPathError",
    "close_all",
    "lower",
    "set",
    "upper",
]
