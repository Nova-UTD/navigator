"""
Builds structured debug dictionaries for publication and logging.
Every lane-change decision cycle is fully explained in the output.
"""

import json
import time

from .types import Scene, LaneChangeNeed, TargetLaneCandidate, GapAssessment, SafetyResult
from .lane_change_state_machine import LCState


class Diagnostics:
    def build(
        self,
        state: LCState,
        prev_state: LCState,
        transition_reason: str,
        scene: Scene,
        need: LaneChangeNeed,
        target: TargetLaneCandidate,
        gap: GapAssessment,
        safety: SafetyResult,
        shadow_mode: bool,
    ) -> dict:
        return {
            "timestamp": round(time.time(), 3),
            "state": state.name,
            "prev_state": prev_state.name,
            "transition_reason": transition_reason,
            "shadow_mode": shadow_mode,
            # Need
            "need_detected": need.needed,
            "need_reason": need.reason,
            "need_urgency": need.urgency,
            "blockage_distance_m": (
                round(need.blockage_distance_m, 2)
                if need.blockage_distance_m != float("inf")
                else -1
            ),
            "suggested_direction": need.suggested_direction,
            # Target lane
            "target_available": target.available,
            "target_direction": target.direction,
            "target_confidence": round(target.confidence, 3),
            "target_lane_id": target.lane_id,
            "target_lateral_offset_m": round(target.lateral_offset_m, 3),
            "target_reason": target.reason,
            # Gap
            "gap_safe": gap.safe,
            "front_gap_m": round(gap.front_gap_m, 2),
            "rear_gap_m": round(gap.rear_gap_m, 2),
            "rear_ttc_s": round(min(gap.rear_ttc_s, 999.0), 2),
            "side_overlap": gap.side_overlap,
            "gap_reason": gap.reason,
            # Safety
            "safety_ok": safety.safe,
            "active_blockers": safety.blockers,
            # Ego
            "ego_speed_mps": round(scene.ego_speed, 2),
            "intersection_stop": scene.intersection_stop,
            "num_path_points": len(scene.current_path),
            "num_objects": len(scene.nearby_objects),
            # Topic health
            "topic_health": {
                "odom": scene.topic_health.odom_fresh,
                "path": scene.topic_health.path_fresh,
                "objects": scene.topic_health.objects_fresh,
                "intersection": scene.topic_health.intersection_fresh,
            },
        }

    def decision_summary(
        self,
        state: LCState,
        need: LaneChangeNeed,
        target: TargetLaneCandidate,
        gap: GapAssessment,
        safety: SafetyResult,
    ) -> dict:
        return {
            "state": state.name,
            "need_detected": need.needed,
            "need_reason": need.reason,
            "target_direction": target.direction,
            "gap_safe": gap.safe,
            "front_gap_m": round(gap.front_gap_m, 2),
            "rear_gap_m": round(gap.rear_gap_m, 2),
            "rear_ttc_s": round(min(gap.rear_ttc_s, 999.0), 2),
            "active_blockers": safety.blockers,
        }

    @staticmethod
    def to_json(d: dict) -> str:
        return json.dumps(d)
