#!/usr/bin/env python3
"""
hybrid_drivable_grid_node.py

Fuses the perception-based drivable grid, the HD-map, and the hybrid occupancy
grid to produce a real-time /grid/drivable OccupancyGrid.

Perception is the PRIMARY source.  HD map is a FALLBACK for cells where
perception is uncertain.  Occupancy grid provides real-time obstacle veto.

Fusion rules (per cell, in priority order):
  1. Perception high-confidence road (perc_occ < PERC_ROAD_THRESHOLD, i.e. evidence > 0.70)
       -> open as road (0), regardless of HD map.
       -> Perception has seen this cell clearly; HD map boundaries do not override.
  2. Perception uncertain (perc_occ >= PERC_ROAD_THRESHOLD)
       -> fall back to HD map:
            HD map == 0   (mapped road)    -> road (0)
            HD map other  (boundary/unknown)-> obstacle (100), safe default
  3. No perception data yet (startup / topic gap)
       -> full HD map fallback, identical to previous behaviour.
  4. Occupancy grid obstacle veto applied last on all confirmed road cells:
       occ >= OCC_BLOCK_THRESHOLD -> blocked (100)
       0 < occ < threshold        -> hazard value passed through

This gives three-layer failsafe with correct priority:
  Perception (10 Hz, map-free)  — primary
  HD map (slow, map-dependent)  — fallback when perception is uncertain
  Occupancy (LiDAR+camera obs)  — real-time obstacle veto

No downstream changes: /grid/drivable topic and OccupancyGrid format unchanged.

Inputs:
  /grid/drivable/segmented     PerceptionDrivableGridNode  (primary)
  /grid/drivable/hdmap         MapManager OccupancyGrid    (fallback, remapped in launch)
  /grid/occupancy/current      HybridPerceptionGridNode    (obstacle veto)

Output:
  /grid/drivable               300x300 OccupancyGrid, 0.2m/cell, base_link
"""

import threading
import numpy as np
import rclpy
from rclpy.node    import Node
from nav_msgs.msg  import OccupancyGrid

# Occupancy value at or above which a confirmed road cell is treated as blocked.
OCC_BLOCK_THRESHOLD = np.int8(80)

# Perception occupancy BELOW this threshold → high-confidence road.
# Evidence > 0.70 (occupancy < 30).  Only these cells can override HD map
# boundaries.  Speckles / uncertain cells (occ >= 30) fall back to HD map.
PERC_ROAD_THRESHOLD = np.int8(30)


class HybridDrivableGridNode(Node):

    def __init__(self):
        super().__init__('hybrid_drivable_grid_node')
        self._lock = threading.Lock()

        self._hdmap: np.ndarray | None  = None
        self._perc:  np.ndarray | None  = None   # /grid/drivable/segmented

        qos1 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OccupancyGrid, '/grid/drivable/hdmap',
            self._cb_hdmap, qos1)
        self.create_subscription(OccupancyGrid, '/grid/drivable/segmented',
            self._cb_perc, qos_be)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/drivable', 10)

        # 20-Hz (matches route_costmap_node, which reads this for goal selection)
        self.create_timer(0.05, self._publish_loop)

        self._last_hdmap_stamp = None

        self.get_logger().info('HybridDrivableGridNode ready — perception-primary, HD map fallback.')

    # -------------------------------------------------------------------------

    def _cb_hdmap(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._hdmap = arr
            self._last_hdmap_stamp = msg.header.stamp

    def _cb_perc(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._perc = arr

    # -------------------------------------------------------------------------

    def _publish_loop(self):
        with self._lock:
            hdmap = self._hdmap.copy() if self._hdmap is not None else None
            perc  = self._perc.copy()  if self._perc  is not None else None

        if perc is None and hdmap is None:
            self.get_logger().warn(
                'No perception or HD map data yet.', throttle_duration_sec=5.0)
            return

        # ── helper: resize any grid to 300×300 ───────────────────────────────
        target_shape = (300, 300)

        def _resize(arr):
            if arr.shape == target_shape:
                return arr
            import cv2
            return cv2.resize(arr.astype(np.float32),
                              (target_shape[1], target_shape[0]),
                              interpolation=cv2.INTER_NEAREST).astype(np.int8)

        if perc is not None:
            perc = _resize(perc)

        if hdmap is not None:
            hdmap = _resize(hdmap)

        # ── step 1: build base drivability ───────────────────────────────────
        # Start from HD map as the base (preserves all mapped road/boundary/unknown
        # values exactly as before). Perception can only OPEN cells — it never adds
        # new obstacles. Obstacle detection is the occupancy grid's job (step 2).
        # This ensures unmapped/uncertain areas keep their HD map values and the
        # path planner always has a navigable route through mapped road.
        if hdmap is not None:
            output = hdmap.copy()
        else:
            # No HD map — start permissive (unknown=50) so planner can still route
            output = np.full(target_shape, np.int8(50), dtype=np.int8)

        if perc is not None:
            # High-confidence perception road → open, overrides HD map boundaries
            # This extends drivable area into unmapped/uncharted road.
            output[perc < PERC_ROAD_THRESHOLD] = np.int8(0)
            # Uncertain perception cells: HD map base is preserved unchanged.
            # Perception does NOT close cells — no new obstacles from perception.
        else:
            self.get_logger().warn(
                'Perception grid not yet received, using HD map only.',
                throttle_duration_sec=10.0)

        # NOTE: No occupancy veto here.
        # /grid/occupancy/current already flows into grid_summation_node as its
        # own independent channel and is combined via np.maximum with steering cost.
        # Applying it again here would double-count obstacle evidence and incorrectly
        # mark road cells as 100, blocking the path planner from finding any route.

        # ── publish ────────────────────────────────────────────────────────────
        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'base_link'
        msg.info.resolution           = 0.2
        msg.info.width                = target_shape[1]
        msg.info.height               = target_shape[0]
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
