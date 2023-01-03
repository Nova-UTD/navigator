# Allows us to reference Road within its own class
from __future__ import annotations

from dataclasses import dataclass
from shapely.geometry import LineString

from .lane_section import LaneSection
from .enums import RoadType


@dataclass
class Road:
    name: str
    id: int
    length: float
    junction: int
    speed_limit: float
    type: RoadType
    next: Road
    prev: Road
    refline: LineString
    lane_offset: LineString
    sections: list[LaneSection]
