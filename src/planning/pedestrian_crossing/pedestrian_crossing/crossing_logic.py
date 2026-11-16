#!/usr/bin/env python3
"""
Package: pedestrian_crossing
File: crossing_logic.py
Author: Anirudh Devatha

Pure crossing-detection logic -- no ROS, no message types.

Everything here operates on plain tuples so it can be unit tested without a
running graph, following the same split navigator_lane_change uses between
its logic modules and lane_change_node.py. pedestrian_crossing_node.py owns
all the ROS I/O and calls into this module.

Two independent classifications are produced per pedestrian:

  intent -- what the pedestrian appears to be doing (PedestrianIntent)
  status -- what the vehicle should do about it (STATUS_CLEAR/SLOW/STOP)

status is primarily driven by time-to-collision against the path corridor,
with confirmed intent able to escalate it.
"""

import math
from enum import IntEnum
from typing import Dict, List, Optional, Sequence, Tuple

# Status values. Kept as plain ints so this module stays importable without
# navigator_msgs; they match the constants in PedestrianStatus.msg.
STATUS_CLEAR = 0
STATUS_SLOW = 1
STATUS_STOP = 2


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------

PATH_CORRIDOR_HALF_WIDTH = 2.5
LOOKAHEAD_DISTANCE = 30.0
APPROACH_ZONE_EXTRA = 3.0

# Minimum speed toward the path to count as "approaching" (m/s).
APPROACH_VELOCITY_THRESHOLD = 0.3

# Cosine of the cone half-angle that counts as "moving toward the path".
APPROACH_DOT_THRESHOLD = 0.3

# Frames a pedestrian must hold a state before the state machine commits.
STATE_CONFIRM_FRAMES = 3

# Frames without detection before a pedestrian is dropped from tracking.
STALE_FRAME_LIMIT = 10

# Trajectory prediction horizon and step size (seconds).
PREDICTION_DT = 0.1
PREDICTION_HORIZON = 5.0
PREDICTION_STEPS = int(round(PREDICTION_HORIZON / PREDICTION_DT))  # 50

# TTC horizons (seconds).
TTC_STOP_THRESHOLD = 2.0   # TTC < 2s  -> STOP  (critical window)
TTC_SLOW_THRESHOLD = 5.0   # TTC 2-5s  -> SLOW  (yielding window)
                           # TTC > 5s  -> CLEAR (planning window)

# Minimum elapsed time between the two samples used for a velocity estimate.
# Below this, single-frame tracker jitter dominates the derivative.
VELOCITY_MIN_DT = 0.15

# Positions retained per pedestrian for velocity estimation.
POSITION_HISTORY_LEN = 10


class PedestrianIntent(IntEnum):
    """What a pedestrian appears to be doing. Values match PedestrianStatus.msg."""

    IRRELEVANT = 0       # Off to the side, not engaging with the road
    UNCERTAIN = 1        # In the approach zone, intent unclear
    INTENT_TO_CROSS = 2  # Facing the road and moving toward it
    CROSSING = 3         # Inside the path corridor
    CLEARED = 4          # Was crossing, has since exited the corridor


# ---------------------------------------------------------------------------
# Path corridor geometry
# ---------------------------------------------------------------------------

Waypoints = Sequence[Tuple[float, float]]
Polygon = List[Tuple[float, float]]


def build_corridor_polygon(
    waypoints: Waypoints,
    half_width: float,
    lookahead: float,
) -> Polygon:
    """
    Build a closed polygon approximating the drivable corridor along the path.

    Waypoints are in the vehicle frame, so the vehicle sits at the origin and
    `lookahead` simply bounds the distance from it.

    Args:
        waypoints:  Path centerline as (x, y) pairs in the vehicle frame.
        half_width: Half-width of the corridor in meters.
        lookahead:  Max distance from the vehicle to include.

    Returns:
        Polygon vertices, left edge forward then right edge back. Empty when
        the path is too short to define a corridor.
    """
    near = [(x, y) for x, y in waypoints if math.hypot(x, y) <= lookahead]
    if len(near) < 2:
        return []

    left_side: Polygon = []
    right_side: Polygon = []

    for i in range(len(near) - 1):
        x0, y0 = near[i]
        x1, y1 = near[i + 1]

        dx = x1 - x0
        dy = y1 - y0
        length = math.hypot(dx, dy)
        if length < 1e-6:
            continue

        # Left-hand normal to the segment.
        nx = -dy / length
        ny = dx / length

        left_side.append((x0 + nx * half_width, y0 + ny * half_width))
        right_side.append((x0 - nx * half_width, y0 - ny * half_width))

        # Cap the corridor at the final waypoint, so the last segment is
        # included rather than the polygon stopping one segment short.
        if i == len(near) - 2:
            left_side.append((x1 + nx * half_width, y1 + ny * half_width))
            right_side.append((x1 - nx * half_width, y1 - ny * half_width))

    if len(left_side) < 2:
        return []

    return left_side + list(reversed(right_side))


def point_in_polygon(px: float, py: float, polygon: Polygon) -> bool:
    """Ray-casting point-in-polygon test."""
    n = len(polygon)
    if n < 3:
        return False

    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = polygon[i]
        xj, yj = polygon[j]
        if (yi > py) != (yj > py):
            x_cross = (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi
            if px < x_cross:
                inside = not inside
        j = i

    return inside


def nearest_path_point(
    waypoints: Waypoints, px: float, py: float
) -> Tuple[float, float]:
    """Nearest waypoint to (px, py), in the same frame."""
    best_dist = float('inf')
    best = (px, py)

    for wx, wy in waypoints:
        dist = math.hypot(wx - px, wy - py)
        if dist < best_dist:
            best_dist = dist
            best = (wx, wy)

    return best


# ---------------------------------------------------------------------------
# Per-pedestrian state
# ---------------------------------------------------------------------------

class PedestrianStateHandler:
    """
    Tracks intent state and time-to-collision for a single pedestrian.

    One instance per unique tracker ID. Mirrors the IntersectionHandler
    pattern in autonomous_cruise: pure logic, no ROS.
    """

    def __init__(self, pedestrian_id: int):
        self.pedestrian_id = pedestrian_id
        self.state = PedestrianIntent.IRRELEVANT

        # Timestamped position history: (t_seconds, x, y).
        self.position_history: List[Tuple[float, float, float]] = []

        # Consecutive frames each candidate state has been observed.
        self._confirm_count: Dict[PedestrianIntent, int] = {
            s: 0 for s in PedestrianIntent
        }

        self.frames_since_seen = 0

        self.last_x: Optional[float] = None
        self.last_y: Optional[float] = None

        # Latest camera contribution, if any.
        self.distance_to_road: Optional[float] = None
        self.camera_corroborated: bool = False

        # Latest TTC solution.
        self.ttc: float = -1.0
        self.predicted_trajectory: List[Tuple[float, float]] = []

    # -- observation -------------------------------------------------------

    def update(
        self,
        x: float,
        y: float,
        stamp: float,
        in_corridor: bool,
        in_approach_zone: bool,
        facing_road: bool = False,
        distance_to_road: Optional[float] = None,
        camera_corroborated: bool = False,
        nearest_path_x: float = 0.0,
        nearest_path_y: float = 0.0,
    ):
        """
        Fold one frame of observation into this pedestrian's state.

        Args:
            x, y:                Position in the path frame.
            stamp:               Observation time in seconds.
            in_corridor:         Inside the path corridor polygon.
            in_approach_zone:    Inside the widened approach polygon.
            facing_road:         Camera says the pedestrian faces the road.
            distance_to_road:    Camera distance to road edge (m), or None.
            camera_corroborated: A camera detection was matched this frame.
            nearest_path_x/y:    Closest path waypoint, for direction checks.
        """
        self.frames_since_seen = 0
        self.distance_to_road = distance_to_road
        self.camera_corroborated = camera_corroborated

        self.last_x = x
        self.last_y = y

        self.position_history.append((stamp, x, y))
        if len(self.position_history) > POSITION_HISTORY_LEN:
            self.position_history.pop(0)

        candidate = self._evaluate_candidate_state(
            in_corridor, in_approach_zone, facing_road,
            nearest_path_x, nearest_path_y,
        )

        for s in PedestrianIntent:
            self._confirm_count[s] = self._confirm_count[s] + 1 if s == candidate else 0

        # A pedestrian entering the corridor commits immediately -- waiting
        # STATE_CONFIRM_FRAMES to admit that someone is physically in the path
        # would trade away the reaction time the confirmation is meant to buy.
        if candidate == PedestrianIntent.CROSSING:
            self.state = candidate
        elif self._confirm_count[candidate] >= STATE_CONFIRM_FRAMES:
            self.state = candidate

    def tick_unseen(self):
        """Record that this pedestrian was not detected this frame."""
        self.frames_since_seen += 1

    def is_stale(self) -> bool:
        """True once this pedestrian has gone unseen for too long."""
        return self.frames_since_seen > STALE_FRAME_LIMIT

    # -- derived quantities ------------------------------------------------

    def velocity(self) -> Optional[Tuple[float, float]]:
        """
        Estimate (vx, vy) in m/s from the timestamped position history.

        Pairs the newest sample with the most recent sample at least
        VELOCITY_MIN_DT older, so the result is in real units regardless of
        the caller's tick rate and is not dominated by per-frame jitter.
        Returns None until there is enough history to divide by.
        """
        if len(self.position_history) < 2:
            return None

        t_new, x_new, y_new = self.position_history[-1]

        for t_old, x_old, y_old in reversed(self.position_history[:-1]):
            dt = t_new - t_old
            if dt >= VELOCITY_MIN_DT:
                return (x_new - x_old) / dt, (y_new - y_old) / dt

        # Nothing old enough yet -- fall back to the full span available.
        t_old, x_old, y_old = self.position_history[0]
        dt = t_new - t_old
        if dt < 1e-6:
            return None
        return (x_new - x_old) / dt, (y_new - y_old) / dt

    def compute_ttc_and_signal(
        self, corridor: Polygon
    ) -> Tuple[float, int, List[Tuple[float, float]]]:
        """
        Project the pedestrian forward at constant velocity and find the first
        predicted position falling inside the path corridor.

        The resulting TTC gives a base status:
            TTC < 2s  -> STOP
            TTC 2-5s  -> SLOW
            otherwise -> CLEAR

        Confirmed intent then escalates it, so a pedestrian standing still in
        the corridor (no velocity, therefore no TTC) is still a STOP, and one
        confirmed to be walking into the road is at least a SLOW before the
        prediction horizon catches them.

        Returns:
            (ttc, status, predicted_trajectory). ttc is -1.0 when no
            intersection is predicted within the horizon.
        """
        vel = self.velocity()

        trajectory: List[Tuple[float, float]] = []
        ttc = -1.0

        if vel is not None and self.last_x is not None and self.last_y is not None:
            vel_x, vel_y = vel
            for step in range(1, PREDICTION_STEPS + 1):
                t = step * PREDICTION_DT
                pred_x = self.last_x + vel_x * t
                pred_y = self.last_y + vel_y * t
                trajectory.append((pred_x, pred_y))

                if ttc < 0 and point_in_polygon(pred_x, pred_y, corridor):
                    ttc = t
                # Keep projecting past the intersection so the published
                # trajectory shows the whole predicted path.

        status = self._escalate_for_intent(status_from_ttc(ttc))

        self.ttc = ttc
        self.predicted_trajectory = trajectory
        return ttc, status, trajectory

    # -- internals ---------------------------------------------------------

    def _evaluate_candidate_state(
        self,
        in_corridor: bool,
        in_approach_zone: bool,
        facing_road: bool,
        nearest_path_x: float,
        nearest_path_y: float,
    ) -> PedestrianIntent:
        """Classify this single frame, before confirmation filtering."""
        if in_corridor:
            return PedestrianIntent.CROSSING

        if self.state == PedestrianIntent.CROSSING:
            return PedestrianIntent.CLEARED

        if in_approach_zone:
            moving_in = self._is_moving_toward_path(nearest_path_x, nearest_path_y)
            # The camera cue promotes to INTENT_TO_CROSS when present, but
            # motion toward the path is sufficient on its own -- the camera
            # layer is optional and usually absent.
            if moving_in and (facing_road or not self.camera_corroborated):
                return PedestrianIntent.INTENT_TO_CROSS
            return PedestrianIntent.UNCERTAIN

        return PedestrianIntent.IRRELEVANT

    def _is_moving_toward_path(
        self, path_nearest_x: float, path_nearest_y: float
    ) -> bool:
        """True when the velocity vector points into the path corridor."""
        vel = self.velocity()
        if vel is None or self.last_x is None or self.last_y is None:
            return False

        vel_x, vel_y = vel
        speed = math.hypot(vel_x, vel_y)
        if speed < APPROACH_VELOCITY_THRESHOLD:
            return False

        to_path_x = path_nearest_x - self.last_x
        to_path_y = path_nearest_y - self.last_y
        to_path_len = math.hypot(to_path_x, to_path_y)
        if to_path_len < 1e-6:
            return True  # Already at the path edge.

        dot = (vel_x / speed) * (to_path_x / to_path_len) + \
              (vel_y / speed) * (to_path_y / to_path_len)

        return dot > APPROACH_DOT_THRESHOLD

    def _escalate_for_intent(self, status: int) -> int:
        """Raise the TTC-derived status when confirmed intent warrants it."""
        if self.state == PedestrianIntent.CROSSING:
            return STATUS_STOP
        if self.state == PedestrianIntent.INTENT_TO_CROSS:
            return max(status, STATUS_SLOW)
        return status


def status_from_ttc(ttc: float) -> int:
    """Map a time-to-collision to a status value."""
    if ttc < 0:
        return STATUS_CLEAR
    if ttc < TTC_STOP_THRESHOLD:
        return STATUS_STOP
    if ttc < TTC_SLOW_THRESHOLD:
        return STATUS_SLOW
    return STATUS_CLEAR
