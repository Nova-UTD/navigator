"""Unit tests for PedestrianHandler -- no ROS required."""

import math

from pedestrian_crossing.pedestrian_handler import (
    COMFORTABLE_DECEL,
    EMERGENCY_DECEL,
    MIN_SLOW_SPEED,
    SIGNAL_CLEAR,
    SIGNAL_SLOW,
    SIGNAL_STOP,
    STOP_HOLD_DURATION,
    PedestrianHandler,
)

CRUISE = 6.0


class FakeStatus:
    """Stand-in for navigator_msgs/PedestrianStatus."""

    def __init__(self, status, ttc=-1.0, pedestrian_id=0):
        self.status = status
        self.ttc = ttc
        self.pedestrian_id = pedestrian_id


class FakeArray:
    """Stand-in for navigator_msgs/PedestrianStatusArray."""

    def __init__(self, *statuses):
        self.statuses = list(statuses)


def clear_msg():
    return FakeArray(FakeStatus(SIGNAL_CLEAR))


# ---------------------------------------------------------------------------
# Baseline
# ---------------------------------------------------------------------------

def test_handler_is_transparent_before_any_message():
    """
    Never having received a message means the pipeline is not running, and
    must not brake the vehicle. Without this, merely building the package
    would put a car into permanent braking.
    """
    h = PedestrianHandler()
    assert h.get_target_speed(CRUISE) == CRUISE
    assert not h.should_hold_brake()
    assert h.compute_pedestrian_deceleration(5.0) is None


def test_never_received_stays_transparent_across_ticks():
    h = PedestrianHandler()
    for i in range(100):
        h.tick(i * 0.1)
    assert h.get_target_speed(CRUISE) == CRUISE
    assert not h.should_hold_brake()


def test_empty_status_array_is_clear():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(), 0.0)
    assert h.get_target_speed(CRUISE) == CRUISE
    assert not h.should_hold_brake()


# ---------------------------------------------------------------------------
# Signal handling
# ---------------------------------------------------------------------------

def test_stop_zeroes_the_target_speed():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 0.0)
    assert h.get_target_speed(CRUISE) == 0.0
    assert h.should_hold_brake()


def test_worst_signal_wins_across_pedestrians():
    h = PedestrianHandler()
    h.update_pedestrian_status(
        FakeArray(
            FakeStatus(SIGNAL_CLEAR, ttc=-1.0, pedestrian_id=1),
            FakeStatus(SIGNAL_SLOW, ttc=4.0, pedestrian_id=2),
            FakeStatus(SIGNAL_STOP, ttc=1.2, pedestrian_id=3),
        ),
        0.0,
    )
    assert h.get_target_speed(CRUISE) == 0.0


def test_most_urgent_ttc_wins_across_pedestrians():
    h = PedestrianHandler()
    h.update_pedestrian_status(
        FakeArray(
            FakeStatus(SIGNAL_SLOW, ttc=4.5, pedestrian_id=1),
            FakeStatus(SIGNAL_SLOW, ttc=2.5, pedestrian_id=2),
        ),
        0.0,
    )
    # 2.5 s is closer to the stop threshold, so it should dominate and give
    # the slower target.
    speed = h.get_target_speed(CRUISE)
    expected = MIN_SLOW_SPEED + (0.5 / 3.0) * (CRUISE - MIN_SLOW_SPEED)
    assert math.isclose(speed, expected, rel_tol=1e-9)


def test_negative_ttc_entries_do_not_poison_the_minimum():
    h = PedestrianHandler()
    h.update_pedestrian_status(
        FakeArray(
            FakeStatus(SIGNAL_CLEAR, ttc=-1.0, pedestrian_id=1),
            FakeStatus(SIGNAL_SLOW, ttc=3.5, pedestrian_id=2),
        ),
        0.0,
    )
    speed = h.get_target_speed(CRUISE)
    expected = MIN_SLOW_SPEED + (1.5 / 3.0) * (CRUISE - MIN_SLOW_SPEED)
    assert math.isclose(speed, expected, rel_tol=1e-9)


# ---------------------------------------------------------------------------
# TTC-proportional slowing
# ---------------------------------------------------------------------------

def test_slow_speed_interpolates_across_the_ttc_window():
    h = PedestrianHandler()

    # At the far edge of the yielding window, barely slowing.
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=5.0)), 0.0)
    assert math.isclose(h.get_target_speed(CRUISE), CRUISE, rel_tol=1e-9)

    # At the near edge, down to the floor.
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=2.0)), 1.0)
    assert math.isclose(h.get_target_speed(CRUISE), MIN_SLOW_SPEED, rel_tol=1e-9)

    # Halfway.
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=3.5)), 2.0)
    midpoint = MIN_SLOW_SPEED + 0.5 * (CRUISE - MIN_SLOW_SPEED)
    assert math.isclose(h.get_target_speed(CRUISE), midpoint, rel_tol=1e-9)


def test_slow_speed_is_monotonic_in_ttc():
    h = PedestrianHandler()
    speeds = []
    for i, ttc in enumerate([2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0]):
        h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=ttc)), i)
        speeds.append(h.get_target_speed(CRUISE))
    assert speeds == sorted(speeds)


def test_slow_ttc_is_clamped_outside_the_window():
    h = PedestrianHandler()
    # SLOW with a TTC beyond the window still cannot exceed cruise.
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=50.0)), 0.0)
    assert h.get_target_speed(CRUISE) <= CRUISE


def test_slow_without_ttc_splits_the_difference():
    """Intent-escalated SLOW carries no TTC; fall back to a mid-range speed."""
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=-1.0)), 0.0)
    expected = (CRUISE + MIN_SLOW_SPEED) / 2.0
    assert math.isclose(h.get_target_speed(CRUISE), expected, rel_tol=1e-9)


# ---------------------------------------------------------------------------
# Hold after stop
# ---------------------------------------------------------------------------

def test_stop_is_held_after_the_pedestrian_clears():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 10.0)
    assert h.should_hold_brake()

    # Pedestrian clears immediately, but the hold keeps the brake on.
    h.update_pedestrian_status(clear_msg(), 10.5)
    assert h.should_hold_brake()
    assert h.get_target_speed(CRUISE) == 0.0

    # Still held just before expiry.
    h.update_pedestrian_status(clear_msg(), 10.0 + STOP_HOLD_DURATION - 0.01)
    assert h.should_hold_brake()

    # Released once the window passes.
    h.update_pedestrian_status(clear_msg(), 10.0 + STOP_HOLD_DURATION + 0.01)
    assert not h.should_hold_brake()
    assert h.get_target_speed(CRUISE) == CRUISE


def test_hold_expires_on_tick_without_new_messages():
    """
    The hold is time-based, so it must release from tick() alone -- a control
    loop running faster than the status topic should not stay pinned to the
    brake just because no new message happened to arrive.
    """
    h = PedestrianHandler(status_timeout=1e9)  # disable failure detection here
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 0.0)
    h.update_pedestrian_status(clear_msg(), 0.1)  # pedestrian steps clear
    assert h.should_hold_brake()                  # ...but the hold is running

    h.tick(STOP_HOLD_DURATION + 0.2)              # no further messages
    assert not h.should_hold_brake()


def test_stop_persists_while_the_last_known_signal_is_stop():
    """
    Hold expiry does not by itself clear a STOP: if the most recent message
    still says STOP, the brake stays on until something says otherwise.
    """
    h = PedestrianHandler(status_timeout=1e9)
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 0.0)

    h.tick(STOP_HOLD_DURATION + 10.0)
    assert h.should_hold_brake()


def test_repeated_stops_extend_the_hold():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 0.0)
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 1.0)

    # Measured from the second STOP, not the first.
    h.update_pedestrian_status(clear_msg(), 1.0 + STOP_HOLD_DURATION - 0.01)
    assert h.should_hold_brake()
    h.update_pedestrian_status(clear_msg(), 1.0 + STOP_HOLD_DURATION + 0.01)
    assert not h.should_hold_brake()


# ---------------------------------------------------------------------------
# Producer failure
# ---------------------------------------------------------------------------

def test_producer_going_silent_after_working_falls_to_stop():
    h = PedestrianHandler(status_timeout=1.0)
    h.update_pedestrian_status(clear_msg(), 0.0)
    assert h.get_target_speed(CRUISE) == CRUISE

    h.tick(0.5)
    assert h.get_target_speed(CRUISE) == CRUISE  # still within the timeout

    h.tick(1.5)
    assert h.should_hold_brake()
    assert h.get_target_speed(CRUISE) == 0.0
    assert h.compute_pedestrian_deceleration(5.0) == EMERGENCY_DECEL


def test_producer_recovery_clears_the_failure():
    h = PedestrianHandler(status_timeout=1.0)
    h.update_pedestrian_status(clear_msg(), 0.0)
    h.tick(2.0)
    assert h.should_hold_brake()

    h.update_pedestrian_status(clear_msg(), 2.1)
    assert not h.should_hold_brake()
    assert h.get_target_speed(CRUISE) == CRUISE


# ---------------------------------------------------------------------------
# Deceleration
# ---------------------------------------------------------------------------

def test_no_deceleration_override_for_clear_or_slow():
    h = PedestrianHandler()
    h.update_pedestrian_status(clear_msg(), 0.0)
    assert h.compute_pedestrian_deceleration(5.0) is None

    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=3.0)), 1.0)
    assert h.compute_pedestrian_deceleration(5.0) is None


def test_deceleration_is_clamped_between_comfortable_and_emergency():
    h = PedestrianHandler()
    for ttc in [0.6, 1.0, 1.5, 1.9]:
        h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=ttc)), ttc)
        decel = h.compute_pedestrian_deceleration(5.0)
        assert EMERGENCY_DECEL <= decel <= COMFORTABLE_DECEL


def test_very_short_ttc_goes_straight_to_emergency():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=0.2)), 0.0)
    assert h.compute_pedestrian_deceleration(5.0) == EMERGENCY_DECEL


def test_unknown_ttc_goes_straight_to_emergency():
    """A STOP with no TTC is the intent-escalated case: assume the worst."""
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=-1.0)), 0.0)
    assert h.compute_pedestrian_deceleration(5.0) == EMERGENCY_DECEL


def test_already_stopped_vehicle_needs_no_deceleration():
    h = PedestrianHandler()
    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 0.0)
    assert h.compute_pedestrian_deceleration(0.0) == 0.0


# ---------------------------------------------------------------------------
# Debug reporting
# ---------------------------------------------------------------------------

def test_signal_name_reports_ttc_and_state():
    h = PedestrianHandler()
    assert h.get_active_signal_name() == 'CLEAR'

    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_SLOW, ttc=3.2)), 0.0)
    assert h.get_active_signal_name() == 'SLOW(TTC=3.2s)'

    h.update_pedestrian_status(FakeArray(FakeStatus(SIGNAL_STOP, ttc=1.0)), 1.0)
    assert h.get_active_signal_name() == 'STOP(TTC=1.0s)'

    # During the post-stop hold the underlying signal is CLEAR again.
    h.update_pedestrian_status(clear_msg(), 1.2)
    assert h.get_active_signal_name() == 'STOP(holding)'


def test_signal_name_reports_producer_failure():
    h = PedestrianHandler(status_timeout=1.0)
    h.update_pedestrian_status(clear_msg(), 0.0)
    h.tick(2.0)
    assert h.get_active_signal_name() == 'STOP(producer failed)'
