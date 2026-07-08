#!/usr/bin/env python3
"""
lane_grid_node.py — lane-indexed BEV grid, built without map_management.

Pipeline (10 Hz, same rate/geometry as perception_drivable_grid_node so the
two grids overlay exactly):
  1. Parse /lidar/filtered ground-level points, score lane-marking-ness by
     LiDAR intensity (lane paint reads brighter than bare asphalt —
     independent of the camera lighting/shadow failures that make
     /grid/drivable/segmented imperfect).
  2. Temporally blend that per-frame marking evidence (same EMA +
     pose-compensation pattern as perception_drivable_grid_node's
     evidence grid) so sparse per-frame LiDAR hits accumulate into stable
     marking lines instead of flickering.
  3. Cut the latest /grid/drivable/segmented mask along those marking
     lines and label the connected components as lanes
     (lane_segmentation.segment_lanes), ordered by lateral (row) position
     at the ego's column.
  4. Publish /grid/lane (navigator_msgs/LaneGrid) with per-cell lane id +
     confidence plus total_lane_count / ego_lane_index / ego_lane_width_m.

/lane_types/detections (lane_type_detector) is used only as a validation
cross-check (both are independently noisy, per-frame heuristics) — logged,
not fused into the published grid.
"""

import math
import threading

import cv2
import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from navigator_msgs.msg import AllLaneDetections, LaneGrid
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2, PointField

from segmentation.bev_geometry import GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y
from segmentation.lane_segmentation import (
    intensity_lane_evidence, segment_lanes, count_and_locate_ego,
)

DRIVABLE_OCC_MAX = 50   # /grid/drivable/segmented cell counts as drivable if occ < this

ALPHA_BLEND = 0.65      # same constant family as perception_drivable_grid_node
VALIDATION_LOG_PERIOD = 10  # log lane-count agreement every N publish ticks (~1 Hz at 10 Hz)

# LaneGrid.msg has no native RViz display (it's a custom message type), so we
# also publish a colorized PointCloud2 debug view on /grid/lane/viz — one
# distinct color per lane id, viewable with a stock rviz_default_plugins/
# PointCloud2 display (Color Transformer: RGB8), same pattern already used
# for /lidar/filtered in navigator_default.rviz.
_LANE_COLORS = np.array([
    (230, 25, 75), (60, 180, 75), (255, 225, 25), (0, 130, 200),
    (245, 130, 48), (145, 30, 180), (70, 240, 240), (240, 50, 230),
], dtype=np.uint8)


def _build_lane_viz_cloud(lane_id_grid, stamp):
    rows, cols = np.nonzero(lane_id_grid >= 0)
    msg = PointCloud2()
    msg.header.stamp = stamp
    msg.header.frame_id = 'base_link'
    msg.height = 1
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 16
    msg.is_dense = True
    if rows.size == 0:
        msg.width = 0
        msg.row_step = 0
        msg.data = b''
        return msg

    xs = (ORIGIN_X + (cols + 0.5) * RESOLUTION).astype(np.float32)
    ys = (ORIGIN_Y + (rows + 0.5) * RESOLUTION).astype(np.float32)
    zs = np.zeros_like(xs)

    lane_ids = lane_id_grid[rows, cols].astype(np.int64)
    palette = _LANE_COLORS[lane_ids % len(_LANE_COLORS)]
    rgb_uint32 = (palette[:, 0].astype(np.uint32) << 16 |
                  palette[:, 1].astype(np.uint32) << 8 |
                  palette[:, 2].astype(np.uint32))
    rgb_float = rgb_uint32.view(np.float32)

    pts = np.column_stack([xs, ys, zs, rgb_float]).astype(np.float32)
    msg.width = pts.shape[0]
    msg.row_step = msg.point_step * pts.shape[0]
    msg.data = pts.tobytes()
    return msg


def _parse_xyzi(msg):
    """Extract an Nx4 (x, y, z, intensity) array from a PointCloud2, reading
    field offsets dynamically (mirrors perception_drivable_grid_node's
    _lidar_evidence_grid) rather than assuming a fixed struct layout.
    Returns None if the cloud has no intensity field."""
    n = msg.width * msg.height
    if n == 0:
        return None
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z', 'intensity'):
            offsets[f.name] = f.offset
    if not {'x', 'y', 'z', 'intensity'} <= offsets.keys():
        return None
    ps = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)
    cols = []
    for name in ('x', 'y', 'z', 'intensity'):
        off = offsets[name]
        cols.append(np.frombuffer(raw[:, off:off + 4].tobytes(), dtype=np.float32))
    points = np.stack(cols, axis=1)
    ok = np.isfinite(points).all(axis=1)
    return points[ok]


class LaneGridNode(Node):

    def __init__(self):
        super().__init__('lane_grid_node')
        self._lock = threading.Lock()

        self._marking_evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        self._drivable_mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)

        self._last_x = self._last_y = self._last_yaw = None
        self._latest_detector_count = None
        self._tick = 0

        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(PointCloud2, '/lidar/filtered', self._cb_lidar, qos_be)
        self.create_subscription(OccupancyGrid, '/grid/drivable/segmented', self._cb_drivable, qos_be)
        self.create_subscription(Odometry, '/gnss/odometry', self._cb_odom, qos_be)
        self.create_subscription(AllLaneDetections, '/lane_types/detections',
                                  self._cb_lane_detections, qos_be)

        self.pub = self.create_publisher(LaneGrid, '/grid/lane', 10)
        self.viz_pub = self.create_publisher(PointCloud2, '/grid/lane/viz', 10)
        self.create_timer(0.05, self._publish_loop)
        self.get_logger().info('LaneGridNode ready — /grid/lane, /grid/lane/viz')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_lidar(self, msg):
        points = _parse_xyzi(msg)
        if points is None:
            return
        frame_marking = intensity_lane_evidence(points)
        with self._lock:
            # Lane markings default to "absent" whether observed or not, so a
            # single EMA (no separate unobserved-decay branch) is enough —
            # unlike drivable-area evidence, there's no neutral prior to hold.
            self._marking_evidence = (ALPHA_BLEND * self._marking_evidence
                                       + (1 - ALPHA_BLEND) * frame_marking)

    def _cb_drivable(self, msg):
        grid = np.array(msg.data, dtype=np.int16).reshape(GRID_SIZE, GRID_SIZE)
        with self._lock:
            self._drivable_mask = grid < DRIVABLE_OCC_MAX

    def _cb_lane_detections(self, msg):
        if not msg.lane_detections:
            return
        with self._lock:
            self._latest_detector_count = msg.lane_detections[-1].totallanecount

    def _cb_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y = p.x, p.y
        _, _, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        with self._lock:
            if self._last_x is None:
                self._last_x, self._last_y, self._last_yaw = x, y, yaw
                return
            self._compensate_pose(x, y, yaw)
            self._last_x, self._last_y, self._last_yaw = x, y, yaw

    def _compensate_pose(self, x, y, yaw):
        """Warp the persistent marking-evidence grid for ego motion between
        frames — identical pattern to perception_drivable_grid_node's
        _compensate_pose, so both grids stay aligned the same way."""
        dx = x - self._last_x
        dy = y - self._last_y
        dyaw = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dx) < 0.02 and abs(dy) < 0.02 and abs(dyaw) < 0.008:
            return
        sc = -dx / RESOLUTION
        sr = -dy / RESOLUTION
        vc = (0.0 - ORIGIN_X) / RESOLUTION
        vr = (0.0 - ORIGIN_Y) / RESOLUTION
        ca, sa = math.cos(-dyaw), math.sin(-dyaw)
        m = np.float32([[ca, -sa, sc + vc * (1 - ca) + vr * sa],
                        [sa, ca, sr - vc * sa + vr * (1 - ca)]])
        self._marking_evidence = cv2.warpAffine(
            self._marking_evidence, m, (GRID_SIZE, GRID_SIZE),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0.0)

    # ── publish loop ──────────────────────────────────────────────────────────

    def _publish_loop(self):
        with self._lock:
            drivable_mask = self._drivable_mask.copy()
            marking_evidence = self._marking_evidence.copy()
            detector_count = self._latest_detector_count

        lane_id_grid, confidence_grid = segment_lanes(drivable_mask, marking_evidence)
        total_lane_count, ego_lane_index, ego_lane_width_m = count_and_locate_ego(lane_id_grid)

        self._tick += 1
        if detector_count is not None and self._tick % VALIDATION_LOG_PERIOD == 0:
            agree = 'yes' if detector_count == total_lane_count else 'no'
            self.get_logger().info(
                f'lane count check — grid={total_lane_count} '
                f'lane_type_detector={detector_count} agree={agree}')

        msg = LaneGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.width = GRID_SIZE
        msg.height = GRID_SIZE
        msg.resolution = RESOLUTION
        msg.origin.position.x = ORIGIN_X
        msg.origin.position.y = ORIGIN_Y
        msg.origin.position.z = 0.0
        msg.origin.orientation.w = 1.0
        msg.cell_lane_id = lane_id_grid.ravel().tolist()
        msg.cell_confidence = confidence_grid.ravel().tolist()
        msg.total_lane_count = total_lane_count
        msg.ego_lane_index = ego_lane_index
        msg.ego_lane_width_m = ego_lane_width_m
        self.pub.publish(msg)

        self.viz_pub.publish(_build_lane_viz_cloud(lane_id_grid, msg.header.stamp))


def main(args=None):
    rclpy.init(args=args)
    node = LaneGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
