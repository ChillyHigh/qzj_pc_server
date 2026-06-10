from __future__ import annotations

import numpy as np

from connection import FLAG_LOWER_FUNNEL_OPEN, FLAG_UPPER_FUNNEL_OPEN
from plan.types import PlannedPath

from . import config


def set(upper_open: bool, lower_open: bool) -> PlannedPath:
    flags = 0
    if upper_open:
        flags |= FLAG_UPPER_FUNNEL_OPEN
    if lower_open:
        flags |= FLAG_LOWER_FUNNEL_OPEN
    duration = config.OPEN_DURATION if flags != 0 else 0.0
    if duration < 0.0:
        raise ValueError(f"flags path duration 不能小于 0：{duration}")

    def sampler(t: np.ndarray, order: int) -> np.ndarray:
        if order == 0:
            value = float(flags)
        elif order == 1:
            value = 0.0
        else:
            return np.zeros_like(t, dtype=float)
        if t.ndim == 0:
            return np.array([value], dtype=float)
        return np.full((len(t), 1), value, dtype=float)

    return PlannedPath(sampler, float(duration))


def upper(open: bool) -> PlannedPath:
    return set(open, False)


def lower(open: bool) -> PlannedPath:
    return set(False, open)


def close_all() -> PlannedPath:
    return set(False, False)


__all__ = [
    "close_all",
    "lower",
    "set",
    "upper",
]
