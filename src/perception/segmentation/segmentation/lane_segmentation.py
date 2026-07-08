#!/usr/bin/env python3
"""
lane_segmentation.py — pure numpy/scipy lane-indexing algorithm.

No ROS/rclpy/cv2 dependency so this can be unit-tested standalone. The ROS
node wrapper (lane_grid_node.py) supplies real sensor data and handles
temporal smoothing; this module only reasons about a single frame.

Grid axis convention (matches perception_drivable_grid_node.py — see its
FOOTPRINT_HALF_W being applied to the row range and FOOTPRINT_REAR/FRONT to
the column range): row = lateral offset, column = longitudinal/forward
offset. Lanes sit side by side across ROWS at a given column, so "which lane
am I in" and "how many lanes are there" are read off a COLUMN slice (fixed
column, varying row) — not a row slice.

Pipeline for one frame:
  1. intensity_lane_evidence — turn ground-level LiDAR intensity into a
     per-cell "lane marking" score. Retro-reflective paint reads back
     brighter than bare asphalt, independent of camera lighting/shadows,
     so this is a channel the drivable grid's camera-driven false
     positives/negatives don't share.
  2. segment_lanes — cut the drivable mask along marking-evidence barriers
     and label the resulting connected components as lanes, ordered by
     increasing row (lateral position) at the ego column.
  3. count_and_locate_ego — read off total lane count, ego's lane index,
     and ego lane width from the labeled grid.
"""

import numpy as np
from scipy.ndimage import label as sp_label

from segmentation.bev_geometry import (
    GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y, VEHICLE_COL, VEHICLE_ROW,
)

# Ground band for LiDAR points considered for lane-marking evidence — same
# convention as perception_drivable_grid_node's LIDAR_GROUND_Z/-0.30 floor.
GROUND_Z_MIN = -0.30
GROUND_Z_MAX = 0.35

MARKING_MIN_HITS  = 2      # need >= 2 ground returns in a cell to trust its intensity ratio
MARKING_PERCENTILE = 85.0  # "bright" = top 15% of intensities in this frame's ground points

MARKING_THRESHOLD = 0.5    # marking_evidence >= this cuts the drivable mask

CONF_ASSIGNED   = 90  # cell belongs to a lane component connected to the ego column
CONF_UNASSIGNED = 30  # cell is drivable but not connected to any ego-column lane
CONF_NONE       = 0   # cell is not drivable


def intensity_lane_evidence(points_xyzi, grid_size=GRID_SIZE):
    """points_xyzi: Nx4 array of (x, y, z, intensity) in base_link.

    Returns a (grid_size, grid_size) float32 grid in [0, 1]: the fraction of
    ground-level LiDAR returns in each cell whose intensity is in the top
    MARKING_PERCENTILE of this frame's ground returns. Cells with fewer than
    MARKING_MIN_HITS ground returns are left at 0 (no evidence either way).
    """
    evidence = np.zeros((grid_size, grid_size), dtype=np.float32)
    if points_xyzi is None or len(points_xyzi) == 0:
        return evidence

    x, y, z, intensity = (points_xyzi[:, 0], points_xyzi[:, 1],
                          points_xyzi[:, 2], points_xyzi[:, 3])
    ground = (z >= GROUND_Z_MIN) & (z < GROUND_Z_MAX) & np.isfinite(x) & np.isfinite(y)
    x, y, intensity = x[ground], y[ground], intensity[ground]
    if len(intensity) == 0:
        return evidence

    gc = ((x - ORIGIN_X) / RESOLUTION).astype(np.int32)  # column = longitudinal (x)
    gr = ((y - ORIGIN_Y) / RESOLUTION).astype(np.int32)  # row = lateral (y)
    inb = (gc >= 0) & (gc < grid_size) & (gr >= 0) & (gr < grid_size)
    gc, gr, intensity = gc[inb], gr[inb], intensity[inb]
    if len(intensity) == 0:
        return evidence

    thresh = np.percentile(intensity, MARKING_PERCENTILE)
    bright = intensity >= thresh

    total_cnt  = np.zeros((grid_size, grid_size), dtype=np.int32)
    bright_cnt = np.zeros((grid_size, grid_size), dtype=np.int32)
    np.add.at(total_cnt, (gr, gc), 1)
    if bright.any():
        np.add.at(bright_cnt, (gr[bright], gc[bright]), 1)

    trusted = total_cnt >= MARKING_MIN_HITS
    with np.errstate(divide='ignore', invalid='ignore'):
        ratio = np.where(trusted, bright_cnt / np.maximum(total_cnt, 1), 0.0)
    evidence[:] = np.clip(ratio, 0.0, 1.0)
    return evidence


def segment_lanes(drivable_mask, marking_evidence,
                   ego_row=VEHICLE_ROW, ego_col=VEHICLE_COL,
                   marking_threshold=MARKING_THRESHOLD):
    """drivable_mask: (H, W) bool, True where the drivable grid says drivable.
    marking_evidence: (H, W) float in [0, 1] from intensity_lane_evidence.

    Cuts drivable_mask along cells where marking_evidence >= marking_threshold,
    labels the remaining connected components (4-connectivity), and assigns
    lane ids by increasing row (lateral position) to whichever components
    touch the ego's column — components that don't touch the ego column
    (e.g. a visible cross-street) are left unassigned (-1) rather than
    guessed at.

    Returns (lane_id_grid int16, confidence_grid uint8), both (H, W).
    """
    h, w = drivable_mask.shape
    lane_id_grid = np.full((h, w), -1, dtype=np.int16)
    confidence_grid = np.zeros((h, w), dtype=np.uint8)

    barrier = marking_evidence >= marking_threshold
    traversable = drivable_mask & ~barrier
    labeled, _ = sp_label(traversable)

    veh_label = labeled[ego_row, ego_col] if traversable[ego_row, ego_col] else 0
    if veh_label == 0:
        # Ego cell itself isn't traversable (e.g. sitting on/near a marking) —
        # fall back to the nearest traversable cell in the ego column.
        col_labels = labeled[:, ego_col]
        nonzero_rows = np.flatnonzero(col_labels)
        if len(nonzero_rows):
            nearest_row = nonzero_rows[np.argmin(np.abs(nonzero_rows - ego_row))]
            veh_label = col_labels[nearest_row]

    confidence_grid[drivable_mask] = CONF_UNASSIGNED

    if veh_label != 0:
        col_labels = labeled[:, ego_col]
        # Increasing-row order of distinct components touching the ego column.
        ordered_labels = []
        seen = set()
        for lbl in col_labels:
            if lbl != 0 and lbl not in seen:
                seen.add(lbl)
                ordered_labels.append(lbl)
        label_to_lane = {lbl: i for i, lbl in enumerate(ordered_labels)}

        for lbl, lane_id in label_to_lane.items():
            mask = labeled == lbl
            lane_id_grid[mask] = lane_id
            confidence_grid[mask] = CONF_ASSIGNED

    confidence_grid[~drivable_mask] = CONF_NONE
    return lane_id_grid, confidence_grid


def count_and_locate_ego(lane_id_grid, ego_row=VEHICLE_ROW, ego_col=VEHICLE_COL,
                          resolution=RESOLUTION):
    """Reads total lane count, ego's lane index, and ego lane width off the
    ego column of a labeled lane_id_grid. Returns -1/-1/0.0 if the ego cell
    isn't assigned to any lane."""
    col = lane_id_grid[:, ego_col]
    valid_ids = sorted(int(v) for v in np.unique(col) if v >= 0)
    total_lane_count = len(valid_ids)

    ego_id = int(lane_id_grid[ego_row, ego_col])
    if ego_id < 0:
        return total_lane_count, -1, 0.0

    ego_lane_index = valid_ids.index(ego_id)
    lane_width_cells = int(np.count_nonzero(col == ego_id))
    lane_width_m = lane_width_cells * resolution
    return total_lane_count, ego_lane_index, lane_width_m
