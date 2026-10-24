#!/usr/bin/env python3
"""
route_costmap_v2_node.py — publishes a second, vision-based route cost
grid for the vehicle's own lane, built from YOLOPv2's drivable-area
segmentation mask (/drivable_mask/front), for a planner to prefer the
lane center rather than just avoid marked-occupied cells.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Naming note: this project already has a route_costmap_node in the
`costs` package (Justin Ruths), which subscribes to the planned
/planning/route path and publishes /grid/route_distance -- a
distance-to-planned-path cost, unrelated to this node's approach and
left completely untouched here. This node is a second, independent way
to get a route cost grid, built purely from live camera segmentation
rather than the planned path, kept side by side for comparison -- hence
the "_v2" in both the node name and its /route_costmap/segmented_v2
topic, to avoid colliding with that existing node's ROS graph name while
still being findable as "the other route costmap."

This is also a distinct output from this package's drivable-AREA grids
(perception_drivable_grid_node.py's /grid/drivable/segmented, and this
package's own yolopv2_drivable_grid_node.py at
/grid/drivable/segmented_v3): those answer "is this cell drivable or
not," a binary occupancy question. This node answers a different
question -- "how good is this cell as a point on our route through the
current lane" -- a graded cost, lowest at the lane center and rising
toward its edges, meant for a planner's cost-based search rather than a
simple binary obstacle check. All three kinds of grid are kept side by
side on purpose; this one does not replace any of them.

Projection (unchanged, shared with yolopv2_drivable_grid_node.py): a
per-pixel camera->ground-plane ray-cast lookup table
(bev_geometry.CamLUT), precomputed once at startup, maps every pixel of
the front camera's drivable-area mask into a BEV grid cell. A cell is
confirmed only once it has enough total projected samples to trust its
drivable/total ratio at all (MIN_SAMPLE_SIZE) and that ratio clears
RATIO_THRESHOLD -- see yolopv2_drivable_grid_node.py's module docstring
for the live tuning history behind both constants (a cell's available
camera pixel budget shrinks sharply with distance under perspective, so
neither a low sample floor nor a high one alone is correct).

From there this node's job diverges: rather than publishing that raw
confirmed mask directly, it turns it into a robust, graded cost field.
See lane_costmap.py for the full reasoning and unit tests behind each
step; in short:

  1. seeded_connected_mask keeps only the connected component touching
     the vehicle's own cell, dropping disconnected noise islands or an
     unrelated lane picked up elsewhere in the frame.
  2. RollingMajorityFilter requires a short window of recent frames to
     agree before trusting a cell, so a few frames of the model
     accidentally bleeding into an adjacent lane can't reach the output.
  3. distance_cost_grid runs a distance transform on that robust region:
     0 at the deepest point (the lane center, found for free as
     whichever cell is farthest from every edge -- no separate
     centerline-tracking step at all), ramping up to 100 at the region's
     boundary, and 100 (fully non-drivable) outside it.

Output is an OccupancyGrid on /route_costmap/segmented_v2, same 0-100
int8 convention as the rest of this project's grids so it drops directly
into RViz's existing Map display and any downstream code already reading
an OccupancyGrid.
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
_OUTPUT_TOPIC = '/route_costmap/segmented_v2'

# See yolopv2_drivable_grid_node.py's module docstring for the live
# tuning history behind these two: a plain absolute pixel-count floor is
# too strict at range (perspective shrinks a cell's camera pixel budget
# with distance), and a plain ratio with too low a floor is unstable on
# tiny sample sizes. Both are needed together.
MIN_SAMPLE_SIZE = 3
RATIO_THRESHOLD = 0.5

# How far around the vehicle's own cell to search for a seed point (see
# lane_costmap.seeded_connected_mask). Confirmed live: the front camera
# has a real blind spot roughly 4m / 20 grid cells directly ahead of the
# vehicle (zero projected pixels land there at all), so the vehicle's
# exact cell is essentially never itself marked drivable -- this radius
# must clear that blind spot with margin or seeding finds nothing every
# frame.
SEED_SEARCH_RADIUS = 25

# At 20Hz, a 5-frame window is a quarter second -- long enough that a
# few-frame mis-segmentation burst can't reach a majority, short enough
# that a real, sustained change (an actual lane change) still wins
# within a fraction of a second.
MAJORITY_WINDOW = 5
MAJORITY_MIN_VOTES = 3

# Cap distance (in grid cells) at which distance_cost_grid's cost
# bottoms out at 0. ~9 cells = 1.8m at this grid's 0.2m resolution,
# roughly half a real lane's width -- a starting point, not a tuned
# final value.
MAX_COST_DIST_CELLS = 9

_FRONT_NAME, _FRONT_TOPIC, _FRONT_T, _FRONT_R = next(c for c in CAMERAS if c[0] == 'front')


class RouteCostmapV2Node(Node):

    def __init__(self):
        super().__init__('route_costmap_v2_node')
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
        self.get_logger().info(f'RouteCostmapV2Node ready — {_OUTPUT_TOPIC}')

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

        seeded = seeded_connected_mask(confirmed, VEHICLE_ROW, VEHICLE_COL,
                                        seed_search_radius=SEED_SEARCH_RADIUS)
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
    node = RouteCostmapV2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
