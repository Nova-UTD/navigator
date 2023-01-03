from __future__ import annotations

from dataclasses import dataclass

from .lane import Lane
from .signals import Signal


@dataclass
class LaneSection:
    s: float
    signals: list[Signal]
    lanes: list[Lane]
