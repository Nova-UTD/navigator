"""Unit tests for NeedDetector — no ROS required."""

import time
import pytest
from navigator_lane_change.need_detector import NeedDetector
from navigator_lane_change.types import (
    Scene, EgoPose, TrackedObject, TopicHealth,
)


def make_scene(objects=None, path=None, intersection_stop=False, ego_speed=0.0):
    if path is None:
        # Straight path along +X axis from origin
        path = [(float(i), 0.0, 0.0) for i in range(0, 100, 2)]
    return Scene(
        stamp=time.time(),
        ego_pose=EgoPose(x=0.0, y=0.0, z=0.0, yaw=0.0),
        ego_speed=ego_speed,
        ego_heading=0.0,
        current_path=path,
        nearby_objects=objects or [],
        intersection_stop=intersection_stop,
        topic_health=TopicHealth(
            odom_fresh=True, path_fresh=True,
            objects_fresh=True, intersection_fresh=True
        ),
    )


def stopped_vehicle(x, y=0.0, speed=0.0):
    return TrackedObject(
        object_id="sv1", classification="vehicle",
        x=x, y=y, z=0.0,
        vx=speed, vy=0.0,
        length=4.5, width=2.0, height=1.5,
    )


class TestNeedDetector:
    def test_no_objects_returns_no_need(self):
        nd = NeedDetector(min_blockage_duration_s=0.0)
        scene = make_scene(objects=[])
        result = nd.detect(scene)
        assert not result.needed
        assert result.reason == "path_clear"

    def test_stopped_vehicle_ahead_triggers_need(self):
        nd = NeedDetector(min_blockage_duration_s=0.0)
        scene = make_scene(objects=[stopped_vehicle(x=15.0)])
        result = nd.detect(scene)
        assert result.needed
        assert result.reason == "stopped_object_blocking_path"
        assert result.blockage_distance_m < 20.0

    def test_moving_vehicle_does_not_trigger(self):
        nd = NeedDetector(min_blockage_duration_s=0.0)
        # Vehicle moving at 5 m/s — above stopped threshold of 0.5
        scene = make_scene(objects=[stopped_vehicle(x=15.0, speed=5.0)])
        result = nd.detect(scene)
        assert not result.needed

    def test_vehicle_behind_ego_does_not_trigger(self):
        nd = NeedDetector(min_blockage_duration_s=0.0)
        scene = make_scene(objects=[stopped_vehicle(x=-10.0)])
        result = nd.detect(scene)
        assert not result.needed

    def test_vehicle_outside_corridor_does_not_trigger(self):
        nd = NeedDetector(
            min_blockage_duration_s=0.0, path_corridor_half_width_m=1.8
        )
        # Vehicle is 5 m to the left — well outside 1.8 m corridor
        scene = make_scene(objects=[stopped_vehicle(x=15.0, y=5.0)])
        result = nd.detect(scene)
        assert not result.needed

    def test_intersection_stop_suppresses_need(self):
        nd = NeedDetector(min_blockage_duration_s=0.0)
        scene = make_scene(
            objects=[stopped_vehicle(x=15.0)],
            intersection_stop=True,
        )
        result = nd.detect(scene)
        assert not result.needed
        assert result.reason == "intersection_stop_active"

    def test_blockage_must_persist_before_trigger(self):
        nd = NeedDetector(min_blockage_duration_s=2.0)
        scene = make_scene(objects=[stopped_vehicle(x=15.0)])
        # First call — not long enough
        result = nd.detect(scene)
        assert not result.needed
        assert result.reason == "blockage_too_brief"

    def test_no_odometry_returns_no_need(self):
        nd = NeedDetector()
        scene = make_scene()
        scene.ego_pose = None
        result = nd.detect(scene)
        assert not result.needed
        assert result.reason == "no_odometry"

    def test_no_path_returns_no_need(self):
        nd = NeedDetector()
        scene = make_scene(path=[])
        result = nd.detect(scene)
        assert not result.needed
        assert result.reason == "no_path"
