"""Unit tests for SafetyChecker — no ROS required."""

import time
from navigator_lane_change.safety_checker import SafetyChecker
from navigator_lane_change.types import (
    Scene, EgoPose, TopicHealth, GapAssessment,
)


def safe_gap():
    return GapAssessment(
        safe=True, front_gap_m=25.0, rear_gap_m=18.0,
        rear_ttc_s=6.0, side_overlap=False, reason="gap_accepted",
    )


def unsafe_gap(reason="rear_ttc_2.0s<4.0s"):
    return GapAssessment(
        safe=False, front_gap_m=25.0, rear_gap_m=18.0,
        rear_ttc_s=2.0, side_overlap=False, reason=reason,
    )


def make_scene(
    odom_fresh=True, path_fresh=True, ego_speed=3.0, intersection_stop=False
):
    path = [(float(i), 0.0, 0.0) for i in range(0, 50, 2)]
    return Scene(
        stamp=time.time(),
        ego_pose=EgoPose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        ego_speed=ego_speed,
        ego_heading=0.0,
        current_path=path,
        nearby_objects=[],
        intersection_stop=intersection_stop,
        topic_health=TopicHealth(
            odom_fresh=odom_fresh, path_fresh=path_fresh,
            objects_fresh=True, intersection_fresh=True,
        ),
    )


class TestSafetyChecker:
    def test_all_ok_returns_safe(self):
        sc = SafetyChecker()
        result = sc.check(make_scene(), safe_gap())
        assert result.safe
        assert result.blockers == []

    def test_stale_odom_blocks(self):
        sc = SafetyChecker()
        result = sc.check(make_scene(odom_fresh=False), safe_gap())
        assert not result.safe
        assert "odom_stale" in result.blockers

    def test_stale_path_blocks(self):
        sc = SafetyChecker()
        result = sc.check(make_scene(path_fresh=False), safe_gap())
        assert not result.safe
        assert "path_stale" in result.blockers

    def test_high_speed_blocks(self):
        sc = SafetyChecker(max_lane_change_speed_mps=5.0)
        result = sc.check(make_scene(ego_speed=9.0), safe_gap())
        assert not result.safe
        assert any("above_max" in b for b in result.blockers)

    def test_intersection_stop_blocks(self):
        sc = SafetyChecker()
        result = sc.check(make_scene(intersection_stop=True), safe_gap())
        assert not result.safe
        assert "intersection_stop_active" in result.blockers

    def test_unsafe_gap_blocks(self):
        sc = SafetyChecker()
        result = sc.check(make_scene(), unsafe_gap())
        assert not result.safe
        assert any("gap_unsafe" in b for b in result.blockers)

    def test_multiple_blockers_all_reported(self):
        sc = SafetyChecker(max_lane_change_speed_mps=5.0)
        result = sc.check(
            make_scene(odom_fresh=False, ego_speed=9.0, intersection_stop=True),
            unsafe_gap(),
        )
        assert not result.safe
        assert len(result.blockers) >= 3
