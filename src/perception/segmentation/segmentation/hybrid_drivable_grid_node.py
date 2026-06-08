#!/usr/bin/env python3
"""
hybrid_drivable_grid_node.py

Fuses the HD-map drivable surface with the hybrid occupancy grid to produce
a real-time /grid/drivable OccupancyGrid.

Instead of re-projecting cameras (which hybrid_perception_grid_node already
does), this node reads the already-computed /grid/occupancy/current and
applies it on top of the MapManager HD-map boundaries.

Fusion rule (per cell):
  HD map == 100 (outside legal lane)  -> always 100, camera cannot override
  HD map ==   0 (inside legal lane):
    occupancy >= occ_threshold        -> mark as blocked (real-time obstacle)
    occupancy ==  0                   -> confirm free
    occupancy ==  -1 (unknown)        -> keep HD map value (0, safe fallback)

Inputs:
  /grid/drivable/hdmap         MapManager OccupancyGrid (remapped in launch)
  /grid/occupancy/current      HybridPerceptionGridNode output

Output:
  /grid/drivable               300x300 OccupancyGrid, 0.2m/cell, base_link
"""

import threading
import numpy as np
import rclpy
from rclpy.node    import Node
from nav_msgs.msg  import OccupancyGrid

# Occupancy value at or above which a road cell is treated as blocked.
# 50 = pole/uncertain, 80 = sidewalk, 100 = hard obstacle.
# Using 80 avoids flagging poles (50) as road-blockers while still
# catching sidewalk encroachments and solid obstacles.
OCC_BLOCK_THRESHOLD = np.int8(80)


class HybridDrivableGridNode(Node):

    def __init__(self):
        super().__init__('hybrid_drivable_grid_node')
        self._lock = threading.Lock()

        self._hdmap: np.ndarray | None  = None   # (H, W) int8 from MapManager
        self._occ:   np.ndarray | None  = None   # (H, W) int8 from occupancy node

        qos1 = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(OccupancyGrid, '/grid/drivable/hdmap',
            self._cb_hdmap, qos1)
        self.create_subscription(OccupancyGrid, '/grid/occupancy/current',
            self._cb_occ, qos1)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/drivable', 10)

        # Publish at 5 Hz — matches occupancy grid update rate (LiDAR+camera)
        self.create_timer(0.2, self._publish_loop)

        self._last_hdmap_stamp = None
        self._last_occ_stamp   = None

        self.get_logger().info('HybridDrivableGridNode ready (occupancy-fusion mode).')

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

    # -------------------------------------------------------------------------

    def _publish_loop(self):
        with self._lock:
            hdmap = self._hdmap.copy() if self._hdmap is not None else None
            occ   = self._occ.copy()   if self._occ   is not None else None

        if hdmap is None:
            self.get_logger().warn(
                'HD map not yet received.', throttle_duration_sec=5.0)
            return

        output = hdmap.copy()   # start from HD map (legal boundary law)

        if occ is not None:
            # Resize occupancy to match hdmap dimensions if needed
            if occ.shape != hdmap.shape:
                import cv2
                occ = cv2.resize(occ.astype(np.float32),
                                 (hdmap.shape[1], hdmap.shape[0]),
                                 interpolation=cv2.INTER_NEAREST).astype(np.int8)

            # Only augment inside-lane cells (hdmap == 0).
            # Hard boundary cells (hdmap == 100) are never touched.
            road = (output == 0)

            # Cells the occupancy grid sees as blocked -> raise to obstacle
            blocked = road & (occ >= OCC_BLOCK_THRESHOLD)
            output[blocked] = np.int8(100)

            # Cells the occupancy grid sees as partially hazardous (1-79)
            # -> pass through the intermediate value so costmap gets nuance
            hazard = road & (occ > 0) & (occ < OCC_BLOCK_THRESHOLD)
            output[hazard] = occ[hazard]

        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'base_link'
        msg.info.resolution           = self._last_hdmap_stamp and 0.2 or 0.2
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
