from __future__ import annotations

from dataclasses import dataclass

from .lane import Lane
from .signals import Signal


@dataclass
class LaneSection:
    s: float = None
    signals: list[Signal] = None
    lanes: list[Lane] = None
    road: Road = None

    def findLane(self, id: int) -> Lane:
        for lane in self.lanes:
            if(lane.id == id):
                return lane
        return None
