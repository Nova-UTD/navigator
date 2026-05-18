"""
Final conservative safety gate — runs before commit and during execution.
Returns a SafetyResult with a list of all active blockers.
"""

from .types import Scene, GapAssessment, SafetyResult


class SafetyChecker:
    def __init__(
        self,
        stale_input_timeout_s: float = 0.5,
        max_lane_change_speed_mps: float = 8.0,
    ):
        self._stale = stale_input_timeout_s
        self._max_speed = max_lane_change_speed_mps

    def check(self, scene: Scene, gap: GapAssessment) -> SafetyResult:
        blockers = []

        if not scene.topic_health.odom_fresh:
            blockers.append("odom_stale")
        if not scene.topic_health.path_fresh:
            blockers.append("path_stale")
        if scene.ego_pose is None:
            blockers.append("ego_pose_missing")
        if len(scene.current_path) < 2:
            blockers.append("path_missing_or_empty")
        if scene.ego_speed > self._max_speed:
            blockers.append(
                f"ego_speed_{scene.ego_speed:.1f}_above_max_{self._max_speed}"
            )
        if scene.intersection_stop:
            blockers.append("intersection_stop_active")
        if not gap.safe:
            blockers.append(f"gap_unsafe:{gap.reason}")

        return SafetyResult(safe=len(blockers) == 0, blockers=blockers)
