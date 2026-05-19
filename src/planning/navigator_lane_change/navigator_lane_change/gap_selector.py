"""
Evaluates whether a safe gap exists in the target lane.

Checks front gap, rear gap, rear TTC, and side overlap.
All calculations are path-relative using Frenet projection.
"""

import math
from typing import Optional, Tuple

from .types import Scene, GapAssessment, TargetLaneCandidate, TrackedObject
from .frenet_utils import project_relative_to_ego

_EPS = 0.1  # m/s — minimum relative speed to avoid divide-by-zero


class GapSelector:
    def __init__(
        self,
        min_front_gap_m: float = 12.0,
        min_rear_gap_m: float = 10.0,
        min_rear_ttc_s: float = 4.0,
        min_side_clearance_m: float = 2.0,
        lookback_m: float = 40.0,
        lookahead_m: float = 60.0,
        merging_conflict_lat_speed_mps: float = 0.4,
    ):
        self._min_front = min_front_gap_m
        self._min_rear = min_rear_gap_m
        self._min_ttc = min_rear_ttc_s
        self._side_clear = min_side_clearance_m
        self._lookback = lookback_m
        self._lookahead = lookahead_m
        self._merge_lat_thresh = merging_conflict_lat_speed_mps

        # Half-width of the band considered "in target lane"
        self._lane_band = 1.6  # m from target lane center

    def assess(self, scene: Scene, target: TargetLaneCandidate) -> GapAssessment:
        if not target.available:
            return self._unsafe("target_lane_not_available")
        if scene.ego_pose is None:
            return self._unsafe("no_ego_pose")
        if len(scene.current_path) < 2:
            return self._unsafe("no_path")

        lat_off = target.lateral_offset_m
        ego = scene.ego_pose
        path = scene.current_path

        front_obj, front_gap = self._front(scene, ego, path, lat_off)
        rear_obj, rear_gap = self._rear(scene, ego, path, lat_off)
        side_overlap = self._side(scene, ego, path, lat_off)
        merge_conflict = self._merging_conflict(scene, ego, path, lat_off)

        # Rear TTC
        if rear_obj is not None:
            closing = rear_obj.speed - scene.ego_speed
            rear_ttc = rear_gap / max(closing, _EPS) if closing > _EPS else 999.0
        else:
            rear_ttc = 999.0

        # Decision
        fails = []
        if front_gap < self._min_front:
            fails.append(f"front_gap_{front_gap:.1f}m<{self._min_front}m")
        if rear_gap < self._min_rear:
            fails.append(f"rear_gap_{rear_gap:.1f}m<{self._min_rear}m")
        if rear_ttc < self._min_ttc:
            fails.append(f"rear_ttc_{rear_ttc:.1f}s<{self._min_ttc}s")
        if side_overlap:
            fails.append("side_overlap")
        if merge_conflict:
            fails.append("merging_conflict_in_target_lane")

        safe = len(fails) == 0
        return GapAssessment(
            safe=safe,
            front_gap_m=front_gap,
            rear_gap_m=rear_gap,
            rear_ttc_s=min(rear_ttc, 999.0),
            side_overlap=side_overlap,
            merging_conflict=merge_conflict,
            reason="gap_accepted" if safe else "; ".join(fails),
        )

    # ------------------------------------------------------------------

    def _front(
        self, scene: Scene, ego, path, lat_off: float
    ) -> Tuple[Optional[TrackedObject], float]:
        best_obj = None
        best_dist = self._lookahead

        for obj in scene.nearby_objects:
            rel_s, lat = project_relative_to_ego(obj.x, obj.y, path, ego.x, ego.y)
            if rel_s <= 0 or rel_s > self._lookahead:
                continue
            if abs(lat - lat_off) > self._lane_band + obj.width / 2.0:
                continue
            if rel_s < best_dist:
                best_dist = rel_s
                best_obj = obj

        return best_obj, best_dist

    def _rear(
        self, scene: Scene, ego, path, lat_off: float
    ) -> Tuple[Optional[TrackedObject], float]:
        best_obj = None
        best_dist = self._lookback

        for obj in scene.nearby_objects:
            rel_s, lat = project_relative_to_ego(obj.x, obj.y, path, ego.x, ego.y)
            if rel_s >= 0 or rel_s < -self._lookback:
                continue
            if abs(lat - lat_off) > self._lane_band + obj.width / 2.0:
                continue
            dist = abs(rel_s)
            if dist < best_dist:
                best_dist = dist
                best_obj = obj

        return best_obj, best_dist

    def _side(self, scene: Scene, ego, path, lat_off: float) -> bool:
        for obj in scene.nearby_objects:
            rel_s, lat = project_relative_to_ego(obj.x, obj.y, path, ego.x, ego.y)
            if abs(rel_s) > 8.0:  # only check objects roughly alongside ego
                continue
            if abs(lat - lat_off) < self._side_clear:
                return True
        return False

    def _merging_conflict(self, scene: Scene, ego, path, lat_off: float) -> bool:
        """True if a vehicle in the target lane is merging laterally toward ego's lane."""
        from .frenet_utils import _path_tangent_at
        ux, uy = _path_tangent_at(ego.x, ego.y, path)
        # Lateral unit vector (positive = left of forward direction)
        lx, ly = -uy, ux

        for obj in scene.nearby_objects:
            rel_s, lat = project_relative_to_ego(obj.x, obj.y, path, ego.x, ego.y)
            if rel_s < -self._lookback or rel_s > self._lookahead:
                continue
            if abs(lat - lat_off) > self._lane_band + obj.width / 2.0:
                continue
            # Project object velocity onto lateral axis
            obj_lat_vel = obj.vx * lx + obj.vy * ly
            # Moving toward ego lane: left-lane object moving right, or right-lane object moving left
            if lat_off > 0 and obj_lat_vel < -self._merge_lat_thresh:
                return True
            if lat_off < 0 and obj_lat_vel > self._merge_lat_thresh:
                return True

        return False

    @staticmethod
    def _unsafe(reason: str) -> GapAssessment:
        return GapAssessment(
            safe=False,
            front_gap_m=0.0,
            rear_gap_m=0.0,
            rear_ttc_s=0.0,
            side_overlap=False,
            reason=reason,
        )
