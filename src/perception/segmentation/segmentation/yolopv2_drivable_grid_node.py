#!/usr/bin/env python3
"""
yolopv2_drivable_grid_node.py — projects YOLOPv2's own drivable-area
segmentation mask (/drivable_mask/front) directly into the BEV grid, via
the same per-pixel camera->ground-plane ray-cast lookup table
(bev_geometry.CamLUT) already used elsewhere in this package.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Supersedes an earlier version of this node that tried to reconstruct a
drivable area purely from YOLOPv2's LANE-LINE head (thin painted markings)
via projection + corridor-fill + curve-following forward extension +
width-based edge inference (lane_bounded_drivable.py, since deleted). That
approach fought a real, fundamental problem the whole way: reconstructing
a filled 2D region from sparse 1-cell-wide line detections is inherently
lossy and heuristic-heavy.

YOLOPv2 is a multi-task model and already has a SEPARATE head trained
directly for drivable-area segmentation -- a dense per-pixel free-space
mask, not lines to reconstruct area from. yolopv2_lane_node.py already
runs that head every frame (one shared forward pass, no extra inference
cost) and already publishes it on /drivable_mask/front -- confirmed live
this session to have a real publisher and be actively computed every
frame, just with zero subscribers until now. This node's whole job is
now just: project that mask into the grid. No corridor-fill, no forward
extension, no line reconstruction -- the model already gives us a filled
region directly.

How the projection works (bev_geometry.CamLUT, unchanged, the same LUT
the lane-line version of this node already used): for every pixel in the
front camera's image, a ray is cast from the camera through that pixel
(via the inverse intrinsic matrix), rotated into the vehicle's base_link
frame using the camera's fixed mounting rotation, and intersected with
the ground plane (z=0) to get a real-world (x, y) point -- this assumes
flat local ground, true for CARLA's road surface. That (x, y) is then
converted to a grid (row, col) via the grid's origin/resolution. This is
precomputed ONCE per camera at startup as a flat per-pixel lookup table
(lut.gr, lut.gc, lut.valid), not recomputed per frame -- projecting a new
mask each frame is then just: for every mask pixel, look up its
precomputed cell and count it there.

Aggregation still needs the same care as the lane-line version's fix: at
range, a grid cell packs in far fewer camera pixels than a cell close to
the vehicle (perspective), so a cell's confirmation can't just be "did
any drivable pixel land here." Each cell needs a MINIMUM SAMPLE SIZE
(MIN_SAMPLE_SIZE) of total projected pixels before trusting its
drivable/total ratio at all, then a RATIO_THRESHOLD on top.

MIN_SAMPLE_SIZE=10 (copied over from the lane-line version's tuning) was
WRONG for this mask, confirmed live: sampling the actual live pixel
density per cell showed cells beyond ~14m forward get only 4-7 total
projected camera pixels even for real, solid road surface -- that never
clears a floor of 10, so everything past 14m was silently discarded
regardless of what the mask said, no matter how confidently the model
had classified it. That floor made sense for the lane-line case (telling
a thin real line apart from noise); it does not apply here, since this
mask is a dense filled region rather than 1-cell-wide lines needing an
anti-noise floor. A live sweep across MIN_SAMPLE_SIZE in {1,2,3,5,10}
found 1,2,3 all recover the same live frame's far-range coverage
identically (1879 confirmed cells past 14m vs 0 at a floor of 10), so 3
was picked: low enough not to blank out real far-range coverage, still
high enough to require more than one stray pixel before trusting a
cell. RATIO_THRESHOLD=0.5 (plain majority vote, appropriate for a dense
region rather than thin lines) changed the same sweep's result only
mildly (0.3/0.5/0.7 -> 1879/1808/1768 far cells on that frame).

STEP 2: once the raw per-frame projection above was confirmed live to be
placed correctly, two more problems showed up that a single frame's mask
can't fix on its own: the raw mask sometimes includes disconnected noise
islands (or even a separate lane, unconnected to ours), and for a few
frames at a time the model can accidentally bleed into an adjacent lane
as if it were ours. Both are handled in lane_costmap.py, kept out of this
node and unit-tested standalone (same pattern as the deleted
lane_bounded_drivable.py): seeded_connected_mask keeps only the connected
component touching the vehicle's own cell, dropping every disconnected
fragment outright; RollingMajorityFilter keeps a short fixed-length
window of recent per-cell masks and requires a majority to agree, so a
brief mis-segmentation burst can't tip a cell on its own, and (unlike an
OR-forever history) nothing lingers once it ages out of the window.

Output is no longer strict binary. distance_cost_grid runs a distance
transform on that robust region and turns it into a 0-100 cost grid: 0 at
the deepest point of the region (the lane center, found for free as
wherever is farthest from every edge -- no separate centerline-tracking
step at all), ramping up to 100 at the region's own boundary, and 100
(fully non-drivable) outside it entirely. Still no LiDAR, no
vehicle-footprint prior.

This is a v3 evaluation node, not a replacement: publishes to
/grid/drivable/segmented_v3, side by side with the existing
/grid/drivable/segmented (perception_drivable_grid_node.py, PSPNet-based,
unchanged) for live comparison before anything downstream is switched
over.
"""

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from segmentation.bev_geometry import (
    CAMERAS, CamLUT, GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y, VEHICLE_ROW, VEHICLE_COL)
from segmentation.lane_costmap import seeded_connected_mask, RollingMajorityFilter, distance_cost_grid

_DRIVABLE_MASK_TOPIC = '/drivable_mask/front'
_OUTPUT_TOPIC = '/grid/drivable/segmented_v3'

# Same reasoning as the lane-line version: a cell's drivable/total ratio
# isn't trustworthy until it has enough total projected samples --
# perspective means far cells get far fewer camera pixels than near ones.
# Confirmed live: 10 (copied from the lane-line tuning) was too strict
# here and silently blanked out everything past ~14m forward, where real
# cells only get 4-7 total projected pixels even for solid road. 3 is the
# lowest floor that still recovered that same far-range coverage in a
# live sweep -- see module docstring.
MIN_SAMPLE_SIZE = 3

# Unlike the sparse 1-cell-wide lane-line case (which needed 0.3 to
# survive thin real features), this mask is a dense filled region, so a
# plain majority vote is the right threshold here.
RATIO_THRESHOLD = 0.5

# RollingMajorityFilter tuning: at 20Hz, a 5-frame window is a quarter
# second -- long enough that a few-frame mis-segmentation burst can't
# reach a majority, short enough that a real, sustained change (an
# actual lane change) still wins within a fraction of a second.
MAJORITY_WINDOW = 5
MAJORITY_MIN_VOTES = 3

# Cap distance (in grid cells) at which distance_cost_grid's cost bottoms
# out at 0. ~9 cells = 1.8m at this grid's 0.2m resolution, roughly half
# a real lane's width -- a starting point, not a tuned final value.
MAX_COST_DIST_CELLS = 9

_FRONT_NAME, _FRONT_TOPIC, _FRONT_T, _FRONT_R = next(c for c in CAMERAS if c[0] == 'front')


class Yolopv2DrivableGridNode(Node):

    def __init__(self):
        super().__init__('yolopv2_drivable_grid_node')
        self.bridge = CvBridge()
        self._lock = threading.Lock()

        self._front_lut = CamLUT(_FRONT_T, _FRONT_R)
        self._latest_drivable_mask = None
        self._majority_filter = RollingMajorityFilter(window=MAJORITY_WINDOW, min_votes=MAJORITY_MIN_VOTES)

        qos_be = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)

        self.create_subscription(Image, _DRIVABLE_MASK_TOPIC, self._cb_drivable_mask, qos_be)

        self.pub = self.create_publisher(OccupancyGrid, _OUTPUT_TOPIC, 10)
        self.create_timer(0.05, self._publish_loop)
        self.get_logger().info(
            f'Yolopv2DrivableGridNode ready (seeded + majority-filtered distance costmap) — {_OUTPUT_TOPIC}')

    # ── callbacks ─────────────────────────────────────────────────────────

    def _cb_drivable_mask(self, msg):
        mask = self.bridge.imgmsg_to_cv2(msg, 'mono8') > 127
        with self._lock:
            self._latest_drivable_mask = mask

    # ── mask → BEV projection ────────────────────────────────────────────

    def _confirmed_area_from_mask(self, drivable_mask):
        """Project one frame's drivable-area pixels into the BEV grid. A
        cell is confirmed only once it has enough total samples to trust
        a ratio at all (MIN_SAMPLE_SIZE) AND that ratio exceeds
        RATIO_THRESHOLD."""
        lut = self._front_lut
        mask = drivable_mask
        if mask.shape[0] != lut.img_h or mask.shape[1] != lut.img_w:
            mask = cv2.resize(mask.astype(np.uint8), (lut.img_w, lut.img_h),
                               interpolation=cv2.INTER_NEAREST).astype(bool)

        total_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)
        bright_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int32)
        valid = lut.valid
        np.add.at(total_cnt, (lut.gr[valid], lut.gc[valid]), 1)
        hit = valid & mask.ravel()
        if hit.any():
            np.add.at(bright_cnt, (lut.gr[hit], lut.gc[hit]), 1)

        enough_samples = total_cnt >= MIN_SAMPLE_SIZE
        with np.errstate(divide='ignore', invalid='ignore'):
            ratio = np.where(enough_samples, bright_cnt / np.maximum(total_cnt, 1), 0.0)
        return enough_samples & (ratio > RATIO_THRESHOLD)

    # ── publish loop ─────────────────────────────────────────────────────

    def _publish_loop(self):
        with self._lock:
            mask_snap = self._latest_drivable_mask

        if mask_snap is None:
            confirmed = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
        else:
            confirmed = self._confirmed_area_from_mask(mask_snap)

        seeded = seeded_connected_mask(confirmed, VEHICLE_ROW, VEHICLE_COL)
        robust = self._majority_filter.update(seeded)
        grid_out = distance_cost_grid(robust, MAX_COST_DIST_CELLS)

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.info.resolution = RESOLUTION
        msg.info.width = GRID_SIZE
        msg.info.height = GRID_SIZE
        msg.info.origin.position.x = ORIGIN_X
        msg.info.origin.position.y = ORIGIN_Y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = grid_out.ravel().tolist()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Yolopv2DrivableGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
