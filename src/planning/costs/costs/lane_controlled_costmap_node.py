"""
Package: costs
   File: lane_controlled_costmap_node.py
 Author: Nova UTD

Publishes /grid/lane_control — a lane-aware cost layer consumed by
grid_summation_node via np.maximum() into /grid/steering_cost.

Grid convention (must match grid_summation_node / path_planner_node):
  frame_id  = base_link  (grid rotates with vehicle — no heading math needed)
  size      = 151 × 151 cells
  resolution= 0.4 m/cell
  origin    = (-20.0, -30.0) in base_link
  ego cell  = (row=75, col=50)
  rows      = lateral axis  (+y = left,  -y = right)
  cols      = longitudinal  (+x = ahead, -x = behind)

Lane-keeping (IDLE / default):
  current lane           →   0  C_FREE       planner traverses freely
  adjacent through lane  →  80  C_HIGH       strong penalty, planner stays put
  turn-only lane         →  87  C_RESTRICTED planner will not cross unless forced
  shoulder / emergency   → 100  C_SHOULDER   treated as obstacle (threshold = 90)

Lane-changing (EXECUTE_LANE_CHANGE):
  target lane            →   0  C_FREE       induces path shift into target lane
  current lane           →  20  C_GRADIENT   gentle gradient, encourages crossing
  all other lanes        → same as IDLE
  safety revert after EXECUTE_TIMEOUT_S → costs revert to IDLE values
"""

import json
import time
import numpy as np
import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String

try:
    from navigator_msgs.msg import AllLaneDetections
    _HAVE_LANE_MSGS = True
except ImportError:
    _HAVE_LANE_MSGS = False

# ── Grid constants ─────────────────────────────────────────────────────────────
GRID_SIZE    = 151
RESOLUTION   = 0.4      # m/cell
ORIGIN_X     = -20.0    # m in base_link  (20 m behind ego)
ORIGIN_Y     = -30.0    # m in base_link  (30 m right of ego)
FRAME_ID     = 'base_link'

EGO_ROW      = 75       # row index of ego lateral centre
LANE_WIDTH_M = 3.5      # m — matches lane_change_node default parameter
LANE_CELLS   = round(LANE_WIDTH_M / RESOLUTION)   # 9 cells
LANE_HALF    = LANE_CELLS // 2                     # 4 cells

# ── Cost values ────────────────────────────────────────────────────────────────
# path_planner_node obstacle_threshold = 90; stay below that for C_RESTRICTED
C_FREE       =   0   # planner traverses freely
C_GRADIENT   =  20   # gentle push away (current lane during EXECUTE)
C_HIGH       =  80   # standard lane-boundary penalty
C_RESTRICTED =  87   # turn-only / restricted lane — very high but not a hard block
C_SHOULDER   = 100   # shoulder / emergency lane — hard obstacle block

# ── Safety: revert to IDLE costs if stuck in EXECUTE too long ─────────────────
EXECUTE_TIMEOUT_S = 10.0

# ── Lane type keyword sets (all lowercased for matching) ──────────────────────
_SHOULDER_KW   = {'shoulder', 'emergency', 'bike lane', 'parking', 'merge'}
_LEFT_ONLY_KW  = {'left turn', 'left only', 'turn left', 'left-turn', 'left-only'}
_RIGHT_ONLY_KW = {'right turn', 'right only', 'turn right', 'right-turn', 'right-only'}

# ── Lane cost lookup ───────────────────────────────────────────────────────────
_LANE_COST = {
    'through':    C_HIGH,
    'left_only':  C_RESTRICTED,
    'right_only': C_RESTRICTED,
    'shoulder':   C_SHOULDER,
}


def _classify_lane(lane_str: str) -> str:
    """Map a raw lane-type string to one of four canonical categories."""
    s = lane_str.lower()
    if any(k in s for k in _SHOULDER_KW):
        return 'shoulder'
    if any(k in s for k in _LEFT_ONLY_KW):
        return 'left_only'
    if any(k in s for k in _RIGHT_ONLY_KW):
        return 'right_only'
    return 'through'


class LaneControlledCostmapNode(Node):

    def __init__(self):
        super().__init__('lane_controlled_costmap_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('publish_rate_hz', 20.0)

        # ── Internal state ────────────────────────────────────────────────────
        self._sim_clock       = Clock()   # rosgraph_msgs Clock — avoids shadowing rclpy.Node._clock
        self._lc_state        = 'IDLE'
        self._lc_direction    = 'none'
        self._execute_entry_t = 0.0   # monotonic clock when EXECUTE was entered

        # {relative_lane_index: lane_category}
        #   0 = ego lane, +N = N lanes left, -N = N lanes right
        self._lane_map: dict = {}

        # ── Subscriptions ─────────────────────────────────────────────────────
        self.create_subscription(Clock, '/clock', self._cb_clock, 1)

        self.create_subscription(
            String, '/behavior/lane_change_decision', self._cb_decision, 10)

        if _HAVE_LANE_MSGS:
            self.create_subscription(
                AllLaneDetections, '/lane_types/detections', self._cb_lanes, 10)
        else:
            self.get_logger().warn(
                'navigator_msgs not found — lane-type restrictions disabled. '
                'All adjacent lanes will be treated as through-lanes (C_HIGH=80).')

        # ── Publisher ─────────────────────────────────────────────────────────
        self._pub = self.create_publisher(OccupancyGrid, '/grid/lane_control', 1)

        # ── Publish timer ─────────────────────────────────────────────────────
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info('LaneControlledCostmapNode ready.')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _cb_clock(self, msg: Clock) -> None:
        self._sim_clock = msg

    def _cb_decision(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except (json.JSONDecodeError, AttributeError):
            return

        new_state = d.get('state', 'IDLE')

        # Record wall-clock entry time when we first see EXECUTE
        if (new_state == 'EXECUTE_LANE_CHANGE'
                and self._lc_state != 'EXECUTE_LANE_CHANGE'):
            self._execute_entry_t = time.monotonic()

        self._lc_state     = new_state
        self._lc_direction = d.get('target_direction', 'none')

    def _cb_lanes(self, msg) -> None:
        """Parse AllLaneDetections into a relative-index → lane-category map."""
        if not msg.lane_detections:
            return

        det = msg.lane_detections[0]   # first detection source
        n_left = int(det.numlanesleftofcurrent)

        new_map = {}
        for abs_idx, lane_str in enumerate(det.alllanes):
            rel_idx = abs_idx - n_left   # 0 = current, + = left, - = right
            new_map[rel_idx] = _classify_lane(lane_str)

        self._lane_map = new_map

    # ── Costmap painting ──────────────────────────────────────────────────────

    def _row_band(self, rel_idx: int):
        """Return inclusive (row_lo, row_hi) for lane at relative index."""
        center = EGO_ROW + rel_idx * LANE_CELLS
        return (
            max(0, center - LANE_HALF),
            min(GRID_SIZE - 1, center + LANE_HALF),
        )

    def _idle_cost(self, rel_idx: int) -> int:
        """Cost for a lane cell in IDLE state, accounting for lane type."""
        if rel_idx == 0:
            return C_FREE
        lane_type = self._lane_map.get(rel_idx, 'through')
        return _LANE_COST.get(lane_type, C_HIGH)

    def _paint(self) -> np.ndarray:
        # Initialise entire grid to C_HIGH.
        # Any row not explicitly covered by a known lane band stays penalised —
        # this is the correct behaviour for territory outside detected lanes.
        grid = np.full((GRID_SIZE, GRID_SIZE), C_HIGH, dtype=np.int16)

        # Safety revert: if EXECUTE has run past the timeout, treat as IDLE.
        executing = (
            self._lc_state == 'EXECUTE_LANE_CHANGE'
            and (time.monotonic() - self._execute_entry_t) < EXECUTE_TIMEOUT_S
        )

        target_rel = 0
        if executing:
            if self._lc_direction == 'left':
                target_rel = +1
            elif self._lc_direction == 'right':
                target_rel = -1

        # Paint every known lane index plus ±2 defaults (safe even without detection).
        indices = set(self._lane_map.keys()) | {-2, -1, 0, 1, 2}

        for rel in indices:
            lo, hi = self._row_band(rel)
            if lo > hi:
                continue   # clamped out-of-bounds

            if executing and rel == target_rel:
                cost = C_FREE
            elif executing and rel == 0:
                cost = C_GRADIENT
            else:
                cost = self._idle_cost(rel)

            grid[lo:hi + 1, :] = cost

        return np.clip(grid, 0, 100).astype(np.int8)

    # ── Timer callback ────────────────────────────────────────────────────────

    def _on_timer(self) -> None:
        arr = self._paint()

        msg = OccupancyGrid()
        msg.header.stamp             = self._sim_clock.clock
        msg.header.frame_id          = FRAME_ID
        msg.info.resolution          = RESOLUTION
        msg.info.width               = GRID_SIZE
        msg.info.height              = GRID_SIZE
        msg.info.origin.position.x   = ORIGIN_X
        msg.info.origin.position.y   = ORIGIN_Y
        msg.info.origin.position.z   = 0.0
        msg.data = arr.flatten().tolist()
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneControlledCostmapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
