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
    speed_limit_ms: float = 10  # meters per second, per ROS standards
    type: RoadType = 'town'
    next: Road = None
    prev: Road = None
    refline: LineString = None
    lane_offset: LineString = None
    sections: list[LaneSection] = None
