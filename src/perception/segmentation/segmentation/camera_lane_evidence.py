#!/usr/bin/env python3
"""
camera_lane_evidence.py — camera-based lane-marking evidence.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Replaces the original LiDAR-intensity approach (intensity_lane_evidence in
lane_segmentation.py, still present but unused by lane_grid_node as of this
change): CARLA's standard sensor.lidar.ray_cast computes intensity from a
pure distance/incidence-angle attenuation model, not from what the ray hit,
so it never actually distinguishes painted lane markings from bare asphalt
in simulation. A real LiDAR's intensity does vary by surface reflectivity,
but that signal doesn't exist here.

Lane paint does reliably read brighter than asphalt in the RGB camera image,
so this module detects it there instead — using a morphological top-hat
filter, which isolates features that are both (a) thinner than the
structuring element and (b) locally brighter than their surroundings, which
is structurally what a lane line is. It stays robust to slow lighting/shadow
gradients across the frame, unlike a single global or even a naive
per-frame-percentile threshold (the flaw that made the LiDAR version fail:
"top 15% of this frame" is always non-empty even when there's no real
signal, since it's relative to the frame's own noise floor rather than any
structural property).

Deliberately classification-agnostic: earlier versions gated the top-hat
detector behind a semantic search mask (road | sidewalk | pole | traffic
sign), extended piecemeal every time live testing found PSPNet mislabeling
real lane paint as yet another Cityscapes class (confirmed live for "pole",
"traffic sign", then "train" -- a fourth, unrelated class). That's a
whack-a-mole pattern with no natural end: PSPNet was never trained to
recognize lane markings, so there's no reason to trust its class boundaries
here. A thin, locally bright line in the middle of the drivable area is a
lane marking regardless of what the classifier calls the pixels around it.
The search mask now only excludes sky (the one class that can never contain
a real lane line, and cheap to rule out) -- classification no longer gates
whether the top-hat detector even looks somewhere. Anything found outside
the actual road corridor is irrelevant anyway -- lane_grid_node only cuts
lanes within the LiDAR-curb-cleaned corridor (road_corridor.py), so a
spurious candidate on a building or vehicle never affects the result.

Pure numpy/opencv, no rclpy — unit-testable standalone, same style as
lane_segmentation.py.
"""

import cv2
import numpy as np

# Match image_segmentation_node._PALETTE (Cityscapes classes)
ROAD_COLOR = (128, 64, 128)
SKY_COLOR = (70, 130, 180)

# Structuring element for the top-hat filter: bigger than an expected lane
# line's pixel width at typical viewing range, smaller than lane width, so
# opening erases the line (letting top-hat recover it) without erasing the
# whole lane surface.
TOPHAT_KERNEL_SIZE = 21
TOPHAT_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                           (TOPHAT_KERNEL_SIZE, TOPHAT_KERNEL_SIZE))

# Starting point only — expect to need live-data tuning once CARLA is back
# up, same as CONFIRMED_THRESHOLD in route_costmap_node needed tonight.
TOPHAT_THRESHOLD = 18  # 0-255 scale; top-hat response above this = candidate marking pixel

MARKING_MIN_HITS = 3  # need >= this many pixels projected into a cell to trust its ratio


def road_mask_from_semantic(semantic_rgb, road_color=ROAD_COLOR):
    """semantic_rgb: HxWx3 uint8, PSPNet's class-colored output (rgb8).

    Returns an HxW bool mask, True where the pixel is classified road.
    """
    return np.all(semantic_rgb == np.array(road_color, dtype=np.uint8), axis=-1)


def marking_search_mask_from_semantic(semantic_rgb):
    """semantic_rgb: HxWx3 uint8, PSPNet's class-colored output (rgb8).

    Returns an HxW bool mask excluding only sky -- see module docstring for
    why classification no longer gates marking detection beyond that. Used
    for marking_candidate_mask specifically, not road_mask_from_semantic
    (which stays a strict road-only match for callers that actually want
    that).
    """
    sky = np.all(semantic_rgb == np.array(SKY_COLOR, dtype=np.uint8), axis=-1)
    return ~sky


def marking_candidate_mask(rgb_image, search_mask,
                            kernel=TOPHAT_KERNEL, tophat_thresh=TOPHAT_THRESHOLD):
    """rgb_image: HxWx3 uint8 raw camera frame. search_mask: HxW bool, e.g.
    from marking_search_mask_from_semantic (everything except sky, see
    module docstring).

    Returns an HxW bool mask of candidate lane-marking pixels: locally
    brighter than their surroundings (top-hat response above threshold)
    AND within the search region.
    """
    gray = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2GRAY)
    tophat = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
    bright = tophat >= tophat_thresh
    return bright & search_mask


def camera_marking_evidence(candidate_mask, cam_lut, grid_size,
                             min_hits=MARKING_MIN_HITS):
    """candidate_mask: HxW bool from marking_candidate_mask (or any HxW bool
    mask with the same shape as the image cam_lut was built for).
    cam_lut: bev_geometry.CamLUT for the same camera/image size.

    Scatters candidate_mask pixels into the (grid_size, grid_size) BEV grid
    via the LUT's precomputed pixel -> cell mapping (same LUT
    perception_drivable_grid_node.py uses, so this stays pixel-aligned with
    the drivable grid for free).

    Returns (evidence, observed), both (grid_size, grid_size):
      evidence: float32 in [0, 1], fraction of valid-projected pixels in
        each cell that were candidates (0 where untrusted or unobserved).
      observed: bool, True where this camera projected >= min_hits pixels
        into the cell. Any one camera only covers part of the grid, so the
        caller must blend evidence into persistent state only where
        observed=True and leave the rest alone -- otherwise a camera frame
        would wrongly decay evidence in cells it never actually looked at
        (e.g. built up moments earlier by a different camera).
    """
    evidence = np.zeros((grid_size, grid_size), dtype=np.float32)
    observed = np.zeros((grid_size, grid_size), dtype=bool)

    valid = cam_lut.valid
    if not valid.any():
        return evidence, observed

    gr = cam_lut.gr[valid]
    gc = cam_lut.gc[valid]
    is_candidate = candidate_mask.ravel()[valid]

    total_cnt = np.zeros((grid_size, grid_size), dtype=np.int32)
    bright_cnt = np.zeros((grid_size, grid_size), dtype=np.int32)
    np.add.at(total_cnt, (gr, gc), 1)
    if is_candidate.any():
        np.add.at(bright_cnt, (gr[is_candidate], gc[is_candidate]), 1)

    observed = total_cnt >= min_hits
    with np.errstate(divide='ignore', invalid='ignore'):
        ratio = np.where(observed, bright_cnt / np.maximum(total_cnt, 1), 0.0)
    evidence[:] = np.clip(ratio, 0.0, 1.0)
    return evidence, observed
