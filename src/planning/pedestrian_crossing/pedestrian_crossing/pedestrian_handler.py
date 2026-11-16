#!/usr/bin/env python3
"""
Package: pedestrian_crossing
File: pedestrian_handler.py
Author: Anirudh Devatha

Pedestrian handling logic for the behavior layer.

Mirrors the IntersectionHandler pattern in
autonomous_cruise/intersection_handler.py: a pure logic class with no ROS
dependencies, fed from a subscription callback and queried once per control
loop tick. It is unit testable without a running graph.

Consumes /pedestrian_status (navigator_msgs/PedestrianStatusArray) published
by pedestrian_crossing_node:

    0 = CLEAR  -- TTC > 5s or no intersection predicted
    1 = SLOW   -- TTC 2-5s, speed reduced proportionally to TTC
    2 = STOP   -- TTC < 2s, or a pedestrian confirmed inside the path corridor

See README.md for the integration snippet.
"""

from typing import Optional


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

SIGNAL_CLEAR = 0
SIGNAL_SLOW = 1
SIGNAL_STOP = 2

# TTC thresholds -- must match pedestrian_crossing_node.py.
TTC_STOP_THRESHOLD = 2.0   # seconds
TTC_SLOW_THRESHOLD = 5.0   # seconds

# Speed floor while SLOW -- never crawl below this (m/s).
MIN_SLOW_SPEED = 1.0

# How long STOP is held after the pedestrian clears (seconds). Prevents
# lurching forward at a pedestrian who has only just stepped out of the path.
STOP_HOLD_DURATION = 1.5

# Comfortable deceleration for SLOW (m/s^2).
COMFORTABLE_DECEL = -1.5

# Emergency deceleration for STOP (m/s^2).
EMERGENCY_DECEL = -3.0

# If no status message arrives for this long after at least one has been
# received, the producer is considered failed (m/s^2 -> seconds).
STATUS_TIMEOUT = 1.0


class PedestrianHandler:
    """
    Converts pedestrian crossing signals into a speed target and, when
    stopping, a deceleration override.

    SLOW speed is proportional to TTC:
        TTC = TTC_SLOW_THRESHOLD (5s) -> cruise speed   (barely slowing)
        TTC = TTC_STOP_THRESHOLD (2s) -> MIN_SLOW_SPEED (near stopped)
        between                       -> linear interpolation

    Failure handling follows the same rule as the producing node: having
    never received a message means the pedestrian pipeline simply is not
    running, and the handler stays transparent (CLEAR). Having received
    messages and then lost them is a real fault, and the handler falls to
    STOP. Without that distinction, merely building this package would put
    a vehicle into permanent braking.
    """

    def __init__(
        self,
        min_slow_speed: float = MIN_SLOW_SPEED,
        stop_hold_duration: float = STOP_HOLD_DURATION,
        comfortable_decel: float = COMFORTABLE_DECEL,
        emergency_decel: float = EMERGENCY_DECEL,
        status_timeout: float = STATUS_TIMEOUT,
    ):
        self.min_slow_speed = min_slow_speed
        self.stop_hold_duration = stop_hold_duration
        self.comfortable_decel = comfortable_decel
        self.emergency_decel = emergency_decel
        self.status_timeout = status_timeout

        # Worst-case signal and most urgent TTC across all pedestrians.
        self._worst_signal: int = SIGNAL_CLEAR
        self._closest_ttc: float = -1.0

        # Hold-after-stop state.
        self._last_stop_time: Optional[float] = None
        self._holding_stop: bool = False

        # Producer liveness.
        self._last_msg_time: Optional[float] = None
        self._producer_failed: bool = False

    # -- input -------------------------------------------------------------

    def update_pedestrian_status(self, msg, current_time: float):
        """
        Fold the latest /pedestrian_status message into internal state.

        Args:
            msg:          PedestrianStatusArray from pedestrian_crossing_node.
            current_time: Current ROS time in seconds.
        """
        self._last_msg_time = current_time
        self._producer_failed = False

        statuses = getattr(msg, 'statuses', None) or []

        if not statuses:
            self._worst_signal = SIGNAL_CLEAR
            self._closest_ttc = -1.0
            self._check_hold_expiry(current_time)
            return

        worst = SIGNAL_CLEAR
        min_ttc = float('inf')

        for status in statuses:
            if status.status > worst:
                worst = status.status
            if 0.0 < status.ttc < min_ttc:
                min_ttc = status.ttc

        self._worst_signal = worst
        self._closest_ttc = min_ttc if min_ttc < float('inf') else -1.0

        if worst == SIGNAL_STOP:
            self._last_stop_time = current_time
            self._holding_stop = True
        else:
            self._check_hold_expiry(current_time)

    def tick(self, current_time: float):
        """
        Advance time-dependent state without a new message.

        Call once per control loop so the post-stop hold expires and producer
        failure is detected even while /pedestrian_status is silent.
        """
        self._check_hold_expiry(current_time)

        if self._last_msg_time is None:
            # Nothing has ever arrived: the pipeline is not running. Stay
            # transparent rather than braking a vehicle for a node that was
            # never launched.
            return

        if (current_time - self._last_msg_time) > self.status_timeout:
            self._producer_failed = True

    # -- output ------------------------------------------------------------

    def get_target_speed(self, cruise_speed: float) -> float:
        """
        Speed target given the current signal.

        CLEAR -> cruise_speed
        SLOW  -> interpolated between min_slow_speed and cruise_speed by TTC
        STOP  -> 0.0
        """
        signal = self._get_effective_signal()

        if signal == SIGNAL_STOP:
            return 0.0
        if signal == SIGNAL_SLOW:
            return self._compute_ttc_proportional_speed(cruise_speed)
        return cruise_speed

    def compute_pedestrian_deceleration(self, current_speed: float) -> Optional[float]:
        """
        Deceleration override while STOP is active.

        Estimates the distance remaining as d = v * TTC and solves
        v^2 = 2*a*d for a, clamped between the comfortable and emergency
        rates. Returns None for CLEAR and SLOW, where the existing PID
        handles the approach smoothly on its own.

        Args:
            current_speed: Current vehicle speed (m/s).

        Returns:
            Deceleration in m/s^2 (negative), or None when not applicable.
        """
        if self._get_effective_signal() != SIGNAL_STOP:
            return None

        if current_speed < 0.1:
            return 0.0

        ttc = self._closest_ttc

        # Unknown or extremely short TTC -- and any producer failure, where
        # there is no TTC to reason about at all -- goes straight to emergency.
        if self._producer_failed or ttc < 0 or ttc < 0.5:
            return self.emergency_decel

        estimated_distance = current_speed * ttc
        if estimated_distance < 1e-6:
            return self.emergency_decel

        required_decel = -(current_speed ** 2) / (2.0 * estimated_distance)

        # Clamp into [emergency, comfortable]: never gentler than comfortable,
        # never harsher than emergency.
        return min(max(required_decel, self.emergency_decel), self.comfortable_decel)

    def should_hold_brake(self) -> bool:
        """True while the brake should be held (STOP, or the post-stop hold)."""
        return self._get_effective_signal() == SIGNAL_STOP

    def get_active_signal_name(self) -> str:
        """Human-readable signal and TTC, for debug logging."""
        if self._producer_failed:
            return 'STOP(producer failed)'

        signal = self._get_effective_signal()
        name = {
            SIGNAL_CLEAR: 'CLEAR',
            SIGNAL_SLOW: 'SLOW',
            SIGNAL_STOP: 'STOP',
        }.get(signal, '?')

        if self._holding_stop and self._worst_signal != SIGNAL_STOP:
            return f'{name}(holding)'
        if self._closest_ttc > 0:
            return f'{name}(TTC={self._closest_ttc:.1f}s)'
        return name

    # -- internals ---------------------------------------------------------

    def _get_effective_signal(self) -> int:
        """Current signal, accounting for producer failure and the stop hold."""
        if self._producer_failed:
            return SIGNAL_STOP
        if self._holding_stop:
            return SIGNAL_STOP
        return self._worst_signal

    def _check_hold_expiry(self, current_time: float):
        """Release the post-stop hold once enough time has passed."""
        if not self._holding_stop or self._last_stop_time is None:
            return
        if (current_time - self._last_stop_time) >= self.stop_hold_duration:
            self._holding_stop = False

    def _compute_ttc_proportional_speed(self, cruise_speed: float) -> float:
        """
        Interpolate between min_slow_speed and cruise_speed by TTC.

            TTC >= TTC_SLOW_THRESHOLD -> cruise_speed
            TTC <= TTC_STOP_THRESHOLD -> min_slow_speed
        """
        ttc = self._closest_ttc

        # SLOW with no TTC means intent-based escalation from the node rather
        # than a predicted intersection. Split the difference.
        if ttc < 0:
            return (cruise_speed + self.min_slow_speed) / 2.0

        ttc = max(TTC_STOP_THRESHOLD, min(ttc, TTC_SLOW_THRESHOLD))
        t = (ttc - TTC_STOP_THRESHOLD) / (TTC_SLOW_THRESHOLD - TTC_STOP_THRESHOLD)

        return self.min_slow_speed + t * (cruise_speed - self.min_slow_speed)
