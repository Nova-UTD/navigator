"""Unit tests for LaneChangeStateMachine — no ROS required."""

import time
import pytest
from navigator_lane_change.lane_change_state_machine import (
    LaneChangeStateMachine, LCState,
)
from navigator_lane_change.types import (
    Scene, EgoPose, TopicHealth,
    LaneChangeNeed, TargetLaneCandidate, GapAssessment, SafetyResult,
)


# ------------------------------------------------------------------ helpers

def make_scene(odom_fresh=True, path_fresh=True):
    return Scene(
        stamp=time.time(),
        ego_pose=EgoPose(0.0, 0.0, 0.0, 0.0),
        ego_speed=3.0, ego_heading=0.0,
        current_path=[(float(i), 0.0, 0.0) for i in range(0, 50, 2)],
        nearby_objects=[],
        intersection_stop=False,
        topic_health=TopicHealth(
            odom_fresh=odom_fresh, path_fresh=path_fresh,
            objects_fresh=True, intersection_fresh=True,
        ),
    )


def no_need():
    return LaneChangeNeed(
        needed=False, reason="path_clear", urgency="none",
        blockage_distance_m=float("inf"), suggested_direction="none",
    )


def need():
    return LaneChangeNeed(
        needed=True, reason="stopped_object_blocking_path", urgency="normal",
        blockage_distance_m=15.0, suggested_direction="left",
    )


def target_ok():
    return TargetLaneCandidate(
        available=True, direction="left", lane_id="adjacent_left",
        confidence=0.88, lateral_offset_m=3.5, reason="clear",
    )


def target_unavail():
    return TargetLaneCandidate(
        available=False, direction="none", lane_id="",
        confidence=0.0, lateral_offset_m=0.0, reason="no_lane",
    )


def gap_safe():
    return GapAssessment(
        safe=True, front_gap_m=25.0, rear_gap_m=18.0,
        rear_ttc_s=6.5, side_overlap=False, reason="gap_accepted",
    )


def gap_unsafe():
    return GapAssessment(
        safe=False, front_gap_m=25.0, rear_gap_m=5.0,
        rear_ttc_s=1.5, side_overlap=False, reason="rear_gap too small",
    )


def safety_ok():
    return SafetyResult(safe=True, blockers=[])


def safety_gap_fail():
    return SafetyResult(safe=False, blockers=["gap_unsafe:rear_ttc_1.5s<4.0s"])


def safety_hard_fail():
    return SafetyResult(safe=False, blockers=["odom_stale"])


# ------------------------------------------------------------------ tests

class TestStateMachine:
    def test_initial_state_is_idle(self):
        sm = LaneChangeStateMachine()
        assert sm.state == LCState.IDLE

    def test_idle_to_prepare_when_need_detected(self):
        sm = LaneChangeStateMachine()
        sm.update(make_scene(), need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.PREPARE_LANE_CHANGE

    def test_idle_stays_idle_when_no_need(self):
        sm = LaneChangeStateMachine()
        sm.update(make_scene(), no_need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.IDLE

    def test_full_happy_path_idle_to_execute(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        # IDLE → PREPARE
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.PREPARE_LANE_CHANGE
        # PREPARE → CHECK_TARGET_LANE
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.CHECK_TARGET_LANE
        # CHECK → FIND_GAP
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.FIND_GAP
        # FIND_GAP → COMMIT
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.COMMIT_LANE_CHANGE
        # COMMIT → EXECUTE
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.EXECUTE_LANE_CHANGE

    def test_stale_inputs_cause_fault(self):
        sm = LaneChangeStateMachine()
        sm.update(make_scene(odom_fresh=False), need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.FAULT

    def test_fault_recovers_to_idle_when_inputs_return(self):
        sm = LaneChangeStateMachine()
        sm.update(make_scene(odom_fresh=False), need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.FAULT
        sm.update(make_scene(odom_fresh=True), no_need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.IDLE

    def test_no_target_lane_goes_to_route_blocked(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        sm.update(scene, need(), target_unavail(), gap_unsafe(), safety_gap_fail())
        # IDLE → PREPARE
        assert sm.state == LCState.PREPARE_LANE_CHANGE
        sm.update(scene, need(), target_unavail(), gap_unsafe(), safety_gap_fail())
        # PREPARE → ROUTE_BLOCKED (no target)
        assert sm.state == LCState.ROUTE_BLOCKED

    def test_safety_hard_failure_during_prepare_goes_to_fault(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        # Now hard failure (odom stale) appears
        sm.update(make_scene(odom_fresh=False), need(), target_ok(), gap_safe(), safety_hard_fail())
        assert sm.state == LCState.FAULT

    def test_safety_failure_during_execute_aborts(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        # Fast-forward to EXECUTE
        for _ in range(5):
            sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.EXECUTE_LANE_CHANGE
        # Safety failure during execution
        sm.update(scene, need(), target_ok(), gap_unsafe(), safety_gap_fail())
        assert sm.state == LCState.ABORT_LANE_CHANGE

    def test_mark_complete_transitions_to_complete(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        for _ in range(5):
            sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.EXECUTE_LANE_CHANGE
        sm.mark_complete()
        assert sm.state == LCState.COMPLETE_LANE_CHANGE

    def test_need_cleared_during_prepare_returns_to_idle(self):
        sm = LaneChangeStateMachine()
        scene = make_scene()
        sm.update(scene, need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.PREPARE_LANE_CHANGE
        sm.update(scene, no_need(), target_ok(), gap_safe(), safety_ok())
        assert sm.state == LCState.IDLE
