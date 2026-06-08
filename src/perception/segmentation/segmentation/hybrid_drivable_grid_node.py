#!/usr/bin/env python3
"""
hybrid_drivable_grid_node.py

Fuses the HD-map drivable surface, the hybrid occupancy grid, and the
perception-based drivable grid to produce a real-time /grid/drivable
OccupancyGrid.

Fusion rules (per cell, in priority order):
  1. HD map == 100 (outside legal lane)
       -> always 100.  Camera/perception cannot override legal boundaries.
  2. HD map ==   0 (inside legal lane):
       a. occupancy >= OCC_BLOCK_THRESHOLD (real obstacle inside lane)
            -> mark as blocked (100)
       b. occupancy in (0, OCC_BLOCK_THRESHOLD) (partial hazard inside lane)
            -> pass through occupancy value for costmap nuance
       c. otherwise
            -> keep 0 (road confirmed by HD map)
  3. HD map != 0 and != 100 (uncharted / unknown area):
       -> If perception grid says high-confidence road (perc_occ < PERC_ROAD_THRESHOLD)
            -> open as road (0) so planner can use uncharted but visually confirmed roads
       -> Otherwise keep HD map value (safe: do not open unconfirmed areas)

This means:
  - The HD map remains the authoritative source for all mapped areas.
  - The perception grid acts as a fallback/extension for unmapped areas only.
  - Speckles or uncertain perception cells (occupancy >= PERC_ROAD_THRESHOLD)
    never open uncharted cells, so noise cannot create false road.
  - Obstacle detection in mapped road areas is unchanged (HybridPerceptionGridNode
    -> /grid/occupancy/current is still the sole obstacle source).
  - No downstream changes needed: /grid/drivable topic and message format unchanged.

Inputs:
  /grid/drivable/hdmap         MapManager OccupancyGrid (remapped in launch)
  /grid/occupancy/current      HybridPerceptionGridNode output
  /grid/drivable/segmented     PerceptionDrivableGridNode output (new)

Output:
  /grid/drivable               300x300 OccupancyGrid, 0.2m/cell, base_link
"""

import threading
import numpy as np
import rclpy
from rclpy.node    import Node
from nav_msgs.msg  import OccupancyGrid

# Occupancy value at or above which a road cell is treated as blocked.
OCC_BLOCK_THRESHOLD = np.int8(80)

# Perception occupancy must be BELOW this to open an uncharted cell as road.
# Corresponds to evidence > 0.70 — only high-confidence perception road cells
# can extend the drivable area beyond HD map coverage.
# Speckles and uncertain cells (occupancy >= 30) are ignored.
PERC_ROAD_THRESHOLD = np.int8(30)


class HybridDrivableGridNode(Node):

    def __init__(self):
        super().__init__('hybrid_drivable_grid_node')
        self._lock = threading.Lock()

        self._hdmap: np.ndarray | None  = None
        self._occ:   np.ndarray | None  = None
        self._perc:  np.ndarray | None  = None   # /grid/drivable/segmented

        qos1 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OccupancyGrid, '/grid/drivable/hdmap',
            self._cb_hdmap, qos1)
        self.create_subscription(OccupancyGrid, '/grid/occupancy/current',
            self._cb_occ, qos1)
        self.create_subscription(OccupancyGrid, '/grid/drivable/segmented',
            self._cb_perc, qos_be)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/drivable', 10)

        self.create_timer(0.2, self._publish_loop)

        self._last_hdmap_stamp = None
        self._last_occ_stamp   = None

        self.get_logger().info('HybridDrivableGridNode ready (HD map + occupancy + perception).')

    # -------------------------------------------------------------------------

    def _cb_hdmap(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._hdmap = arr
            self._last_hdmap_stamp = msg.header.stamp

    def _cb_occ(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._occ = arr
            self._last_occ_stamp = msg.header.stamp

    def _cb_perc(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._perc = arr

    # -------------------------------------------------------------------------

    def _publish_loop(self):
        with self._lock:
            hdmap = self._hdmap.copy() if self._hdmap is not None else None
            occ   = self._occ.copy()   if self._occ   is not None else None
            perc  = self._perc.copy()  if self._perc  is not None else None

        if hdmap is None:
            self.get_logger().warn(
                'HD map not yet received.', throttle_duration_sec=5.0)
            return

        output = hdmap.copy()

        # ── helper: resize grid to hdmap shape if needed ──────────────────────
        def _resize(arr):
            if arr.shape == hdmap.shape:
                return arr
            import cv2
            return cv2.resize(arr.astype(np.float32),
                              (hdmap.shape[1], hdmap.shape[0]),
                              interpolation=cv2.INTER_NEAREST).astype(np.int8)

        # ── step 1: perception extends uncharted areas ─────────────────────────
        # Must run before occupancy so that opened cells can then be blocked
        # by real obstacles in the same pass.
        if perc is not None:
            perc = _resize(perc)
            # Only touch cells that the HD map has not confirmed as road (!=0)
            # and has not marked as hard boundary (!=100).
            uncharted = (hdmap != 0) & (hdmap != np.int8(100))
            # Open cell only when perception has high confidence (low occupancy)
            perc_road = uncharted & (perc < PERC_ROAD_THRESHOLD)
            output[perc_road] = np.int8(0)

        # ── step 2: occupancy blocks/hazards inside confirmed road ─────────────
        if occ is not None:
            occ = _resize(occ)
            road = (output == 0)   # includes cells just opened by perception
            blocked = road & (occ >= OCC_BLOCK_THRESHOLD)
            output[blocked] = np.int8(100)
            hazard = road & (occ > 0) & (occ < OCC_BLOCK_THRESHOLD)
            output[hazard] = occ[hazard]

        # ── publish ────────────────────────────────────────────────────────────
        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'base_link'
        msg.info.resolution           = 0.2
        msg.info.width                = output.shape[1]
        msg.info.height               = output.shape[0]
        msg.info.origin.position.x    = -20.0
        msg.info.origin.position.y    = -30.0
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data                      = output.ravel().tolist()

        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Publish failed: {e}',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = HybridDrivableGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
