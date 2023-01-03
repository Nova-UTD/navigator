from __future__ import annotations

from dataclasses import dataclass
from shapely.geometry import Polygon

from .enums import LaneType
from .lane_section import LaneSection
from .road import Road


@dataclass
class Lane:
    lsec: LaneSection
    road: Road
    type: LaneType
    predecessors: list[Lane]
    successors: list[Lane]
    shape: Polygon
