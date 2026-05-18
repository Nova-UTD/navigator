"""
Determines whether a lane change should be considered.

Triggers on:
  - Stopped / slow obstacle inside the path corridor ahead of ego
  - Obstacle blocked for at least min_blockage_duration_s
"""

import time
from typing import Optional, Tuple

from .types import Scene, LaneChangeNeed, TrackedObject
from .frenet_utils import project_relative_to_ego


class NeedDetector:
    def __init__(
        self,
        lookahead_distance_m: float = 35.0,
        blocked_path_distance_m: float = 20.0,
        stopped_object_speed_mps: float = 0.5,
        min_blockage_duration_s: float = 1.0,
        path_corridor_half_width_m: float = 1.8,
    ):
        self._lookahead = lookahead_distance_m
        self._blocked_dist = blocked_path_distance_m
        self._stopped_spd = stopped_object_speed_mps
        self._min_duration = min_blockage_duration_s
        self._corridor_hw = path_corridor_half_width_m

        self._blockage_first_seen: Optional[float] = None
        self._blocking_id: Optional[str] = None

    def detect(self, scene: Scene) -> LaneChangeNeed:
        if scene.ego_pose is None:
            return self._no("no_odometry")
        if len(scene.current_path) < 2:
            return self._no("no_path")
        if scene.intersection_stop:
            self._reset()
            return self._no("intersection_stop_active")

        blocking, dist = self._find_blocking_object(scene)

        if blocking is None:
            self._reset()
            return self._no("path_clear")

        # Require the blockage to persist before triggering
        if self._blocking_id != blocking.object_id:
            self._blockage_first_seen = time.time()
            self._blocking_id = blocking.object_id

        elapsed = time.time() - self._blockage_first_seen
        if elapsed < self._min_duration:
            return self._no("blockage_too_brief")

        direction = "left"  # TargetLaneSelector will override with best option
        return LaneChangeNeed(
            needed=True,
            reason="stopped_object_blocking_path",
            urgency="normal",
            blockage_distance_m=dist,
            suggested_direction=direction,
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_blocking_object(
        self, scene: Scene
    ) -> Tuple[Optional[TrackedObject], float]:
        ego = scene.ego_pose
        path = scene.current_path
        best_obj: Optional[TrackedObject] = None
        best_dist = float("inf")

        for obj in scene.nearby_objects:
            if obj.speed > self._stopped_spd:
                continue  # Moving vehicle — ACC handles it

            rel_s, lat = project_relative_to_ego(
                obj.x, obj.y, path, ego.x, ego.y
            )

            if rel_s <= 0.5:
                continue  # At or behind ego
            if rel_s > self._lookahead:
                continue  # Too far ahead to act now

            # Account for object's own width in corridor check
            half_obj_w = obj.width / 2.0
            if abs(lat) > self._corridor_hw + half_obj_w:
                continue  # Outside lane corridor

            if rel_s < best_dist:
                best_dist = rel_s
                best_obj = obj

        return best_obj, best_dist

    def _reset(self):
        self._blockage_first_seen = None
        self._blocking_id = None

    @staticmethod
    def _no(reason: str) -> LaneChangeNeed:
        return LaneChangeNeed(
            needed=False,
            reason=reason,
            urgency="none",
            blockage_distance_m=float("inf"),
            suggested_direction="none",
        )
