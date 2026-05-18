"""
Lane-change behavior state machine.

States follow the spec in lane_change_integration_plan.md §6.7.
Every transition is logged with a reason string for bag-based debugging.
"""

import time
from enum import Enum, auto

from .types import Scene, LaneChangeNeed, TargetLaneCandidate, GapAssessment, SafetyResult


class LCState(Enum):
    IDLE = auto()
    PREPARE_LANE_CHANGE = auto()
    CHECK_TARGET_LANE = auto()
    FIND_GAP = auto()
    WAIT_FOR_GAP = auto()
    COMMIT_LANE_CHANGE = auto()
    EXECUTE_LANE_CHANGE = auto()
    COMPLETE_LANE_CHANGE = auto()
    ABORT_LANE_CHANGE = auto()
    ROUTE_BLOCKED = auto()
    FAULT = auto()


class LaneChangeStateMachine:
    def __init__(
        self,
        wait_for_gap_timeout_s: float = 8.0,
        abort_cooldown_s: float = 3.0,
        complete_stabilization_time_s: float = 1.0,
    ):
        self._gap_timeout = wait_for_gap_timeout_s
        self._abort_cd = abort_cooldown_s
        self._complete_stab = complete_stabilization_time_s

        self.state = LCState.IDLE
        self._prev_state = LCState.IDLE
        self._reason = "initialized"
        self._entry_time = time.time()

    @property
    def prev_state(self) -> LCState:
        return self._prev_state

    @property
    def transition_reason(self) -> str:
        return self._reason

    def update(
        self,
        scene: Scene,
        need: LaneChangeNeed,
        target: TargetLaneCandidate,
        gap: GapAssessment,
        safety: SafetyResult,
    ) -> LCState:

        # Hard-fault: required inputs stale
        if not scene.topic_health.odom_fresh or not scene.topic_health.path_fresh:
            if self.state != LCState.FAULT:
                self._go(LCState.FAULT, "required_inputs_stale")
            return self.state

        state = self.state

        if state == LCState.FAULT:
            self._handle_fault(scene)

        elif state == LCState.ABORT_LANE_CHANGE:
            self._handle_abort(need)

        elif state == LCState.COMPLETE_LANE_CHANGE:
            self._handle_complete()

        elif state == LCState.ROUTE_BLOCKED:
            self._handle_route_blocked(need, target)

        elif state == LCState.IDLE:
            if need.needed:
                self._go(LCState.PREPARE_LANE_CHANGE, need.reason)

        elif state == LCState.PREPARE_LANE_CHANGE:
            self._handle_prepare(need, target, safety)

        elif state == LCState.CHECK_TARGET_LANE:
            self._handle_check(need, target)

        elif state == LCState.FIND_GAP:
            self._handle_find_gap(need, gap, safety)

        elif state == LCState.WAIT_FOR_GAP:
            self._handle_wait_for_gap(need, gap, safety)

        elif state == LCState.COMMIT_LANE_CHANGE:
            self._handle_commit(safety)

        elif state == LCState.EXECUTE_LANE_CHANGE:
            self._handle_execute(safety)

        return self.state

    # ------------------------------------------------------------------
    # Per-state handlers
    # ------------------------------------------------------------------

    def _handle_fault(self, scene: Scene):
        if scene.topic_health.odom_fresh and scene.topic_health.path_fresh:
            self._go(LCState.IDLE, "inputs_recovered")

    def _handle_abort(self, need: LaneChangeNeed):
        if time.time() - self._entry_time >= self._abort_cd:
            if not need.needed:
                self._go(LCState.IDLE, "abort_cooldown_path_clear")
            else:
                self._go(LCState.ROUTE_BLOCKED, "abort_cooldown_still_blocked")

    def _handle_complete(self):
        if time.time() - self._entry_time >= self._complete_stab:
            self._go(LCState.IDLE, "stabilization_complete")

    def _handle_route_blocked(self, need: LaneChangeNeed, target: TargetLaneCandidate):
        if not need.needed:
            self._go(LCState.IDLE, "blockage_cleared")
        elif target.available:
            self._go(LCState.PREPARE_LANE_CHANGE, "new_candidate_appeared")

    def _handle_prepare(
        self,
        need: LaneChangeNeed,
        target: TargetLaneCandidate,
        safety: SafetyResult,
    ):
        if not need.needed:
            self._go(LCState.IDLE, "need_cleared")
        elif self._has_hard_blocker(safety):
            self._go(LCState.FAULT, f"hard_safety_blocker:{safety.blockers}")
        elif target.available:
            self._go(LCState.CHECK_TARGET_LANE, f"candidate_{target.direction}")
        else:
            self._go(LCState.ROUTE_BLOCKED, "no_adjacent_lane_available")

    def _handle_check(self, need: LaneChangeNeed, target: TargetLaneCandidate):
        if not need.needed:
            self._go(LCState.IDLE, "need_cleared")
        elif not target.available:
            self._go(LCState.ABORT_LANE_CHANGE, "target_lane_became_invalid")
        else:
            self._go(LCState.FIND_GAP, "target_lane_valid")

    def _handle_find_gap(
        self,
        need: LaneChangeNeed,
        gap: GapAssessment,
        safety: SafetyResult,
    ):
        if not need.needed:
            self._go(LCState.IDLE, "need_cleared")
        elif self._has_hard_blocker(safety):
            self._go(LCState.ABORT_LANE_CHANGE, f"hard_safety:{safety.blockers}")
        elif gap.safe and safety.safe:
            self._go(LCState.COMMIT_LANE_CHANGE, "safe_gap_found")
        else:
            elapsed = time.time() - self._entry_time
            if elapsed < self._gap_timeout:
                self._go(LCState.WAIT_FOR_GAP, f"gap_unsafe_waiting:{gap.reason}")
            else:
                self._go(LCState.ROUTE_BLOCKED, "find_gap_timeout")

    def _handle_wait_for_gap(
        self,
        need: LaneChangeNeed,
        gap: GapAssessment,
        safety: SafetyResult,
    ):
        if not need.needed:
            self._go(LCState.IDLE, "need_cleared")
        elif gap.safe and safety.safe:
            self._go(LCState.COMMIT_LANE_CHANGE, "gap_opened")
        elif time.time() - self._entry_time > self._gap_timeout:
            self._go(LCState.ABORT_LANE_CHANGE, "wait_for_gap_timeout")

    def _handle_commit(self, safety: SafetyResult):
        if not safety.safe:
            self._go(LCState.ABORT_LANE_CHANGE, f"safety_failed_at_commit:{safety.blockers}")
        else:
            # In shadow mode this immediately moves to EXECUTE (no real actuation).
            # In execution mode the node checks path validity here before transitioning.
            self._go(LCState.EXECUTE_LANE_CHANGE, "committed")

    def _handle_execute(self, safety: SafetyResult):
        if not safety.safe:
            self._go(LCState.ABORT_LANE_CHANGE, f"safety_failed_during_execute:{safety.blockers}")
        # Completion is triggered externally via mark_complete()

    # ------------------------------------------------------------------
    # External triggers
    # ------------------------------------------------------------------

    def mark_complete(self):
        self._go(LCState.COMPLETE_LANE_CHANGE, "target_lane_reached")

    def force_abort(self, reason: str):
        self._go(LCState.ABORT_LANE_CHANGE, reason)

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _go(self, new_state: LCState, reason: str):
        if new_state == self.state:
            return
        self._prev_state = self.state
        self.state = new_state
        self._reason = reason
        self._entry_time = time.time()

    @staticmethod
    def _has_hard_blocker(safety: SafetyResult) -> bool:
        """True when blockers exist beyond just gap unsafety."""
        hard = [b for b in safety.blockers if not b.startswith("gap_unsafe")]
        return len(hard) > 0
