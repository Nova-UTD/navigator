"""
Chooses the best adjacent candidate lane for a lane change.

Priority (obstacle-bypass mode, V1):
  1. Direction suggested by NeedDetector
  2. Opposite adjacent direction
  Reject if too many objects in that lateral band.
"""

import math
from typing import List

from .types import Scene, TargetLaneCandidate, TrackedObject
from .frenet_utils import project_relative_to_ego


class TargetLaneSelector:
    def __init__(
        self,
        lane_width_m: float = 3.5,
        allow_left: bool = True,
        allow_right: bool = True,
        require_same_direction: bool = True,
        min_confidence: float = 0.6,
        scan_range_ahead_m: float = 60.0,
        scan_range_behind_m: float = 30.0,
    ):
        self._lane_w = lane_width_m
        self._allow_left = allow_left
        self._allow_right = allow_right
        self._min_conf = min_confidence
        self._scan_ahead = scan_range_ahead_m
        self._scan_behind = scan_range_behind_m

    def select(
        self, scene: Scene, suggested_direction: str = "left"
    ) -> TargetLaneCandidate:
        if scene.ego_pose is None:
            return self._unavail("no_ego_pose")
        if len(scene.current_path) < 2:
            return self._unavail("no_path")

        ordered = self._direction_order(suggested_direction)
        if not ordered:
            return self._unavail("no_direction_allowed")

        for direction in ordered:
            candidate = self._evaluate(scene, direction)
            if candidate.available and candidate.confidence >= self._min_conf:
                return candidate

        return self._unavail("no_adjacent_lane_above_confidence_threshold")

    # ------------------------------------------------------------------

    def _direction_order(self, suggested: str) -> List[str]:
        directions = []
        if suggested == "left":
            if self._allow_left:
                directions.append("left")
            if self._allow_right:
                directions.append("right")
        else:
            if self._allow_right:
                directions.append("right")
            if self._allow_left:
                directions.append("left")
        return directions

    def _evaluate(self, scene: Scene, direction: str) -> TargetLaneCandidate:
        lateral_offset = self._lane_w if direction == "left" else -self._lane_w
        nearby = self._objects_near_target_lane(scene, lateral_offset)

        # Confidence degrades with object count in the target lane band
        if len(nearby) == 0:
            confidence = 0.88
            reason = f"{direction}_adjacent_clear"
        elif len(nearby) <= 2:
            confidence = 0.72
            reason = f"{direction}_adjacent_sparse_{len(nearby)}_objects"
        else:
            confidence = 0.30
            reason = f"{direction}_adjacent_dense_{len(nearby)}_objects"

        return TargetLaneCandidate(
            available=True,
            direction=direction,
            lane_id=f"adjacent_{direction}",
            confidence=confidence,
            lateral_offset_m=lateral_offset,
            reason=reason,
        )

    def _objects_near_target_lane(
        self, scene: Scene, lateral_offset: float
    ) -> List[TrackedObject]:
        ego = scene.ego_pose
        path = scene.current_path
        half_band = self._lane_w / 2.0
        result = []

        for obj in scene.nearby_objects:
            rel_s, lat = project_relative_to_ego(
                obj.x, obj.y, path, ego.x, ego.y
            )
            if rel_s < -self._scan_behind or rel_s > self._scan_ahead:
                continue
            if abs(lat - lateral_offset) <= half_band:
                result.append(obj)

        return result

    @staticmethod
    def _unavail(reason: str) -> TargetLaneCandidate:
        return TargetLaneCandidate(
            available=False,
            direction="none",
            lane_id="",
            confidence=0.0,
            lateral_offset_m=0.0,
            reason=reason,
        )
