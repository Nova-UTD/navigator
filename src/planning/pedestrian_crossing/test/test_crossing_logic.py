"""Unit tests for crossing_logic -- no ROS required."""

import math

from pedestrian_crossing.crossing_logic import (
    PREDICTION_DT,
    STATE_CONFIRM_FRAMES,
    STALE_FRAME_LIMIT,
    STATUS_CLEAR,
    STATUS_SLOW,
    STATUS_STOP,
    PedestrianIntent,
    PedestrianStateHandler,
    build_corridor_polygon,
    nearest_path_point,
    point_in_polygon,
    status_from_ttc,
)


# A 40 m straight path down +x, one waypoint every 2 m.
STRAIGHT_PATH = [(float(x), 0.0) for x in range(0, 42, 2)]


def straight_corridor(half_width=2.5, lookahead=30.0):
    return build_corridor_polygon(STRAIGHT_PATH, half_width, lookahead)


# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

def test_corridor_is_empty_for_degenerate_paths():
    assert build_corridor_polygon([], 2.5, 30.0) == []
    assert build_corridor_polygon([(0.0, 0.0)], 2.5, 30.0) == []
    # Every waypoint beyond the lookahead is dropped.
    assert build_corridor_polygon([(90.0, 0.0), (95.0, 0.0)], 2.5, 30.0) == []


def test_corridor_spans_full_half_width():
    corridor = straight_corridor(half_width=2.5)
    assert corridor

    # Straight path down +x: corridor covers |y| < 2.5 and nothing past it.
    assert point_in_polygon(10.0, 0.0, corridor)
    assert point_in_polygon(10.0, 2.4, corridor)
    assert point_in_polygon(10.0, -2.4, corridor)
    assert not point_in_polygon(10.0, 2.6, corridor)
    assert not point_in_polygon(10.0, -2.6, corridor)


def test_corridor_is_bounded_by_lookahead():
    corridor = straight_corridor(lookahead=30.0)
    assert point_in_polygon(25.0, 0.0, corridor)
    assert not point_in_polygon(35.0, 0.0, corridor)


def test_corridor_includes_its_final_segment():
    # The corridor must reach the last in-range waypoint (30 m), not stop one
    # segment short of it at 28 m.
    corridor = straight_corridor(lookahead=30.0)
    assert point_in_polygon(29.5, 0.0, corridor)


def test_approach_zone_is_wider_than_the_corridor():
    corridor = straight_corridor(half_width=2.5)
    approach = straight_corridor(half_width=5.5)

    # A pedestrian on the shoulder is in the approach zone but not the path.
    assert not point_in_polygon(10.0, 4.0, corridor)
    assert point_in_polygon(10.0, 4.0, approach)


def test_point_in_polygon_rejects_degenerate_polygons():
    assert not point_in_polygon(0.0, 0.0, [])
    assert not point_in_polygon(0.0, 0.0, [(0.0, 0.0), (1.0, 1.0)])


def test_nearest_path_point_finds_closest_waypoint():
    assert nearest_path_point(STRAIGHT_PATH, 10.4, 5.0) == (10.0, 0.0)
    assert nearest_path_point(STRAIGHT_PATH, -5.0, 0.0) == (0.0, 0.0)


def test_nearest_path_point_on_empty_path_returns_the_query():
    assert nearest_path_point([], 3.0, 4.0) == (3.0, 4.0)


# ---------------------------------------------------------------------------
# TTC mapping
# ---------------------------------------------------------------------------

def test_status_from_ttc_thresholds():
    assert status_from_ttc(-1.0) == STATUS_CLEAR   # no intersection
    assert status_from_ttc(0.5) == STATUS_STOP
    assert status_from_ttc(1.9) == STATUS_STOP
    assert status_from_ttc(2.0) == STATUS_SLOW     # boundary belongs to SLOW
    assert status_from_ttc(4.9) == STATUS_SLOW
    assert status_from_ttc(5.0) == STATUS_CLEAR    # beyond the yielding window


# ---------------------------------------------------------------------------
# Velocity estimation
# ---------------------------------------------------------------------------

def feed(handler, samples, **kwargs):
    """Feed (t, x, y) samples through update() with fixed flags."""
    for t, x, y in samples:
        handler.update(x=x, y=y, stamp=t, **kwargs)


def test_velocity_is_none_until_there_is_history():
    h = PedestrianStateHandler(1)
    assert h.velocity() is None

    h.update(x=0.0, y=0.0, stamp=0.0, in_corridor=False, in_approach_zone=False)
    assert h.velocity() is None


def test_velocity_is_in_metres_per_second_not_per_frame():
    # 1 m/s in +y, sampled at 10 Hz for a full second.
    h = PedestrianStateHandler(1)
    samples = [(i * 0.1, 0.0, i * 0.1) for i in range(11)]
    feed(h, samples, in_corridor=False, in_approach_zone=False)

    vx, vy = h.velocity()
    assert math.isclose(vx, 0.0, abs_tol=1e-6)
    # This is the regression that matters: deriving from a fixed frame count
    # instead of elapsed time reported 2x this value.
    assert math.isclose(vy, 1.0, rel_tol=1e-6)


def test_velocity_is_rate_independent():
    """The same real motion sampled at 5 Hz and 20 Hz gives the same speed."""
    slow = PedestrianStateHandler(1)
    feed(slow, [(i * 0.2, i * 0.4, 0.0) for i in range(6)],
         in_corridor=False, in_approach_zone=False)

    fast = PedestrianStateHandler(2)
    feed(fast, [(i * 0.05, i * 0.1, 0.0) for i in range(21)],
         in_corridor=False, in_approach_zone=False)

    assert math.isclose(slow.velocity()[0], 2.0, rel_tol=1e-6)
    assert math.isclose(fast.velocity()[0], 2.0, rel_tol=1e-6)


def test_velocity_handles_duplicate_timestamps():
    h = PedestrianStateHandler(1)
    feed(h, [(1.0, 0.0, 0.0), (1.0, 0.0, 0.0)],
         in_corridor=False, in_approach_zone=False)
    assert h.velocity() is None


# ---------------------------------------------------------------------------
# Intent state machine
# ---------------------------------------------------------------------------

def test_crossing_commits_immediately_without_confirmation_delay():
    """Someone physically in the path is not worth STATE_CONFIRM_FRAMES of doubt."""
    h = PedestrianStateHandler(1)
    h.update(x=10.0, y=0.0, stamp=0.0, in_corridor=True, in_approach_zone=True)
    assert h.state == PedestrianIntent.CROSSING


def test_non_crossing_states_require_confirmation():
    h = PedestrianStateHandler(1)
    for i in range(STATE_CONFIRM_FRAMES - 1):
        h.update(x=10.0, y=4.0, stamp=i * 0.1,
                 in_corridor=False, in_approach_zone=True)
    assert h.state == PedestrianIntent.IRRELEVANT  # not yet confirmed

    h.update(x=10.0, y=4.0, stamp=STATE_CONFIRM_FRAMES * 0.1,
             in_corridor=False, in_approach_zone=True)
    assert h.state == PedestrianIntent.UNCERTAIN


def test_leaving_the_corridor_transitions_to_cleared():
    h = PedestrianStateHandler(1)
    h.update(x=10.0, y=0.0, stamp=0.0, in_corridor=True, in_approach_zone=True)
    assert h.state == PedestrianIntent.CROSSING

    for i in range(STATE_CONFIRM_FRAMES):
        h.update(x=10.0, y=6.0, stamp=(i + 1) * 0.1,
                 in_corridor=False, in_approach_zone=False)
    assert h.state == PedestrianIntent.CLEARED


def test_walking_toward_the_path_confirms_intent_to_cross():
    """Motion into the road alone is enough -- the camera layer is optional."""
    h = PedestrianStateHandler(1)
    # Walking from y=+6 toward y=0 at 1 m/s, alongside x=10.
    for i in range(12):
        t = i * 0.1
        h.update(x=10.0, y=6.0 - t, stamp=t,
                 in_corridor=False, in_approach_zone=True,
                 nearest_path_x=10.0, nearest_path_y=0.0)
    assert h.state == PedestrianIntent.INTENT_TO_CROSS


def test_walking_away_from_the_path_stays_uncertain():
    h = PedestrianStateHandler(1)
    for i in range(12):
        t = i * 0.1
        h.update(x=10.0, y=4.0 + t, stamp=t,
                 in_corridor=False, in_approach_zone=True,
                 nearest_path_x=10.0, nearest_path_y=0.0)
    assert h.state == PedestrianIntent.UNCERTAIN


def test_loitering_pedestrian_stays_uncertain():
    """Below APPROACH_VELOCITY_THRESHOLD is not 'approaching'."""
    h = PedestrianStateHandler(1)
    for i in range(12):
        t = i * 0.1
        h.update(x=10.0, y=4.0 - t * 0.05, stamp=t,   # 0.05 m/s
                 in_corridor=False, in_approach_zone=True,
                 nearest_path_x=10.0, nearest_path_y=0.0)
    assert h.state == PedestrianIntent.UNCERTAIN


def test_camera_disagreement_downgrades_to_uncertain():
    """When the camera IS matched and says 'not facing road', trust it."""
    h = PedestrianStateHandler(1)
    for i in range(12):
        t = i * 0.1
        h.update(x=10.0, y=6.0 - t, stamp=t,
                 in_corridor=False, in_approach_zone=True,
                 facing_road=False, camera_corroborated=True,
                 nearest_path_x=10.0, nearest_path_y=0.0)
    assert h.state == PedestrianIntent.UNCERTAIN


def test_staleness_tracking():
    h = PedestrianStateHandler(1)
    h.update(x=1.0, y=1.0, stamp=0.0, in_corridor=False, in_approach_zone=False)
    assert not h.is_stale()

    for _ in range(STALE_FRAME_LIMIT + 1):
        h.tick_unseen()
    assert h.is_stale()

    h.update(x=1.0, y=1.0, stamp=1.0, in_corridor=False, in_approach_zone=False)
    assert not h.is_stale()  # a fresh sighting resets the counter


# ---------------------------------------------------------------------------
# TTC and status
# ---------------------------------------------------------------------------

def test_pedestrian_walking_into_the_path_yields_a_finite_ttc():
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)

    # Starting 8 m to the left of the path at x=15, walking in at 2 m/s.
    # The corridor edge is at y=2.5, so contact is ~2.75 s away.
    for i in range(6):
        t = i * 0.1
        h.update(x=15.0, y=8.0 - 2.0 * t, stamp=t,
                 in_corridor=False, in_approach_zone=False,
                 nearest_path_x=15.0, nearest_path_y=0.0)

    ttc, status, trajectory = h.compute_ttc_and_signal(corridor)
    assert ttc > 0
    assert 2.0 < ttc < 3.5
    assert status == STATUS_SLOW
    assert len(trajectory) == int(round(5.0 / PREDICTION_DT))


def test_pedestrian_about_to_step_into_the_path_is_a_stop():
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)

    # 1 m outside the corridor edge, walking in at 2 m/s -> ~0.5 s.
    for i in range(6):
        t = i * 0.1
        h.update(x=15.0, y=3.5 - 2.0 * t, stamp=t,
                 in_corridor=False, in_approach_zone=True,
                 nearest_path_x=15.0, nearest_path_y=0.0)

    ttc, status, _ = h.compute_ttc_and_signal(corridor)
    assert 0 < ttc < 2.0
    assert status == STATUS_STOP


def test_pedestrian_walking_parallel_never_intersects():
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)

    # Walking down +x at 1 m/s, 8 m off to the side -- never enters.
    for i in range(11):
        t = i * 0.1
        h.update(x=10.0 + t, y=8.0, stamp=t,
                 in_corridor=False, in_approach_zone=False,
                 nearest_path_x=10.0, nearest_path_y=0.0)

    ttc, status, _ = h.compute_ttc_and_signal(corridor)
    assert ttc == -1.0
    assert status == STATUS_CLEAR


def test_stationary_pedestrian_standing_in_the_path_is_a_stop():
    """
    A motionless pedestrian generates no TTC intersection from velocity alone.
    The CROSSING escalation is what keeps this from reading as CLEAR.
    """
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)

    for i in range(4):
        h.update(x=15.0, y=0.0, stamp=i * 0.1,
                 in_corridor=True, in_approach_zone=True)

    _, status, _ = h.compute_ttc_and_signal(corridor)
    assert h.state == PedestrianIntent.CROSSING
    assert status == STATUS_STOP


def test_cold_start_pedestrian_in_the_path_is_a_stop():
    """Even on the very first frame, before any velocity history exists."""
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)
    h.update(x=15.0, y=0.0, stamp=0.0, in_corridor=True, in_approach_zone=True)

    ttc, status, trajectory = h.compute_ttc_and_signal(corridor)
    assert ttc == -1.0        # no velocity yet
    assert trajectory == []
    assert status == STATUS_STOP


def test_confirmed_intent_escalates_clear_to_slow():
    """
    Someone confirmed walking into the road, but still further out than the
    5 s horizon, should register as SLOW rather than CLEAR.
    """
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)

    # 20 m off to the side, drifting in at 0.5 m/s: ~35 s from the corridor,
    # far beyond the prediction horizon.
    for i in range(12):
        t = i * 0.1
        h.update(x=15.0, y=20.0 - 0.5 * t, stamp=t,
                 in_corridor=False, in_approach_zone=True,
                 nearest_path_x=15.0, nearest_path_y=0.0)

    ttc, status, _ = h.compute_ttc_and_signal(corridor)
    assert ttc == -1.0                              # outside the horizon
    assert h.state == PedestrianIntent.INTENT_TO_CROSS
    assert status == STATUS_SLOW                    # escalated, not CLEAR


def test_irrelevant_pedestrian_is_not_escalated():
    corridor = straight_corridor()
    h = PedestrianStateHandler(1)
    for i in range(6):
        h.update(x=15.0, y=20.0, stamp=i * 0.1,
                 in_corridor=False, in_approach_zone=False)

    _, status, _ = h.compute_ttc_and_signal(corridor)
    assert h.state == PedestrianIntent.IRRELEVANT
    assert status == STATUS_CLEAR
