from dataclasses import dataclass, field
from typing import List, Optional, Tuple
import math


@dataclass
class EgoPose:
    x: float
    y: float
    z: float
    yaw: float  # radians


@dataclass
class TrackedObject:
    object_id: str
    classification: str
    x: float
    y: float
    z: float
    vx: float
    vy: float
    length: float
    width: float
    height: float
    confidence: float = 1.0

    @property
    def speed(self) -> float:
        return math.sqrt(self.vx ** 2 + self.vy ** 2)


@dataclass
class TopicHealth:
    odom_fresh: bool = False
    path_fresh: bool = False
    objects_fresh: bool = False
    intersection_fresh: bool = False


@dataclass
class Scene:
    stamp: float
    ego_pose: Optional[EgoPose]
    ego_speed: float
    ego_heading: float
    current_path: List[Tuple[float, float, float]]  # (x, y, z)
    nearby_objects: List[TrackedObject]
    intersection_stop: bool
    topic_health: TopicHealth
    commanded_direction: str = "none"  # "left" | "right" | "cancel" | "none"


@dataclass
class LaneChangeNeed:
    needed: bool
    reason: str
    urgency: str  # "none" | "normal" | "high"
    blockage_distance_m: float
    suggested_direction: str  # "left" | "right" | "none"


@dataclass
class TargetLaneCandidate:
    available: bool
    direction: str  # "left" | "right" | "none"
    lane_id: str
    confidence: float
    lateral_offset_m: float
    reason: str


@dataclass
class GapAssessment:
    safe: bool
    front_gap_m: float
    rear_gap_m: float
    rear_ttc_s: float
    side_overlap: bool
    reason: str
    merging_conflict: bool = False


@dataclass
class SafetyResult:
    safe: bool
    blockers: List[str] = field(default_factory=list)
