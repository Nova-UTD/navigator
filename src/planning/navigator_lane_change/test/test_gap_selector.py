"""Unit tests for GapSelector — no ROS required."""

import time
import pytest
from navigator_lane_change.gap_selector import GapSelector
from navigator_lane_change.types import (
    Scene, EgoPose, TrackedObject, TopicHealth, TargetLaneCandidate,
)


def make_scene(objects=None, ego_speed=3.0):
    path = [(float(i), 0.0, 0.0) for i in range(0, 100, 2)]
    return Scene(
        stamp=time.time(),
        ego_pose=EgoPose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        ego_speed=ego_speed,
        ego_heading=0.0,
        current_path=path,
        nearby_objects=objects or [],
        intersection_stop=False,
        topic_health=TopicHealth(
            odom_fresh=True, path_fresh=True,
            objects_fresh=True, intersection_fresh=True,
        ),
    )


def left_target():
    return TargetLaneCandidate(
        available=True, direction="left",
        lane_id="adjacent_left", confidence=0.85,
        lateral_offset_m=3.5, reason="clear",
    )


def vehicle(x, y=3.5, vx=0.0, vy=0.0):
    return TrackedObject(
        object_id="v1", classification="vehicle",
        x=x, y=y, z=0.0,
        vx=vx, vy=vy,
        length=4.5, width=2.0, height=1.5,
    )


class TestGapSelector:
    def test_empty_target_lane_is_safe(self):
        gs = GapSelector()
        scene = make_scene(objects=[])
        result = gs.assess(scene, left_target())
        assert result.safe

    def test_front_gap_too_small_rejects(self):
        gs = GapSelector(min_front_gap_m=12.0)
        # Vehicle in target lane 8 m ahead
        scene = make_scene(objects=[vehicle(x=8.0, y=3.5)])
        result = gs.assess(scene, left_target())
        assert not result.safe
        assert "front_gap" in result.reason

    def test_rear_gap_too_small_rejects(self):
        gs = GapSelector(min_rear_gap_m=10.0)
        # Vehicle 5 m behind in target lane
        scene = make_scene(objects=[vehicle(x=-5.0, y=3.5)])
        result = gs.assess(scene, left_target())
        assert not result.safe
        assert "rear_gap" in result.reason

    def test_rear_ttc_too_low_rejects(self):
        gs = GapSelector(min_rear_gap_m=10.0, min_rear_ttc_s=4.0)
        # Vehicle 15 m behind but closing at 10 m/s → TTC = 1.5 s
        scene = make_scene(
            objects=[vehicle(x=-15.0, y=3.5, vx=10.0)],  # approaching fast
            ego_speed=3.0,
        )
        result = gs.assess(scene, left_target())
        assert not result.safe
        assert "rear_ttc" in result.reason

    def test_large_front_and_rear_gap_is_safe(self):
        gs = GapSelector(min_front_gap_m=12.0, min_rear_gap_m=10.0, min_rear_ttc_s=4.0)
        objs = [
            vehicle(x=30.0, y=3.5),   # front: 30 m ahead
            vehicle(x=-20.0, y=3.5),   # rear: 20 m behind, stationary → TTC = inf
        ]
        scene = make_scene(objects=objs, ego_speed=3.0)
        result = gs.assess(scene, left_target())
        assert result.safe
        assert result.front_gap_m >= 12.0
        assert result.rear_gap_m >= 10.0

    def test_unavailable_target_lane_is_unsafe(self):
        gs = GapSelector()
        unavail = TargetLaneCandidate(
            available=False, direction="none", lane_id="",
            confidence=0.0, lateral_offset_m=0.0, reason="no lane",
        )
        result = gs.assess(make_scene(), unavail)
        assert not result.safe

    def test_no_ego_pose_is_unsafe(self):
        gs = GapSelector()
        scene = make_scene()
        scene.ego_pose = None
        result = gs.assess(scene, left_target())
        assert not result.safe

    def test_objects_in_current_lane_dont_fail_gap(self):
        gs = GapSelector(min_rear_gap_m=10.0)
        # Object is in current lane (y=0), not target lane (y=3.5)
        scene = make_scene(objects=[vehicle(x=-5.0, y=0.0)])
        result = gs.assess(scene, left_target())
        # rear gap check should not trigger for current-lane objects
        assert result.rear_gap_m > 10.0
