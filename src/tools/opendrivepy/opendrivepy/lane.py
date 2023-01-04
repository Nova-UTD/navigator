from __future__ import annotations

from dataclasses import dataclass
from shapely.geometry import Polygon, LineString

from .enums import LaneType
# from .lane_section import LaneSection
# from .road import Road


@dataclass
class Lane:
    lsec: LaneSection
    road: Road
    id: int
    type: LaneType = None
    predecessors: list[Lane] = None
    successors: list[Lane] = None
    shape: Polygon = None
    left_bound: LineString = None
    right_bound: LineString = None
