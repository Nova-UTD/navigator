"""Unit tests for lane_segmentation.py — pure numpy/scipy, no ROS required.

Run with: python3 -m pytest src/perception/segmentation/test/test_lane_segmentation.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation.bev_geometry import GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y, VEHICLE_COL, VEHICLE_ROW
from segmentation import lane_segmentation as ls


def make_three_lane_drivable_mask(lane_width_cells=15, gap_cells=1):
    """3 lanes side by side across rows, centered on VEHICLE_ROW, spanning
    all columns. Returns (drivable_mask, lane_row_ranges) where
    lane_row_ranges is a list of (start, end) row ranges, one per lane,
    ego-lane in the middle."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    lane_row_ranges = []
    r = VEHICLE_ROW - (3 * lane_width_cells + 2 * gap_cells) // 2
    for _ in range(3):
        start = r
        end = r + lane_width_cells
        mask[start:end, :] = True
        lane_row_ranges.append((start, end))
        r = end + gap_cells
    return mask, lane_row_ranges


def marking_lines_at_gaps(lane_row_ranges, grid_size=GRID_SIZE):
    """Synthetic marking-evidence grid: a barrier row between each lane gap."""
    evidence = np.zeros((grid_size, grid_size), dtype=np.float32)
    for i in range(len(lane_row_ranges) - 1):
        boundary_row = lane_row_ranges[i][1]  # the gap row between lane i and i+1
        evidence[boundary_row, :] = 1.0
    return evidence


def test_intensity_lane_evidence_flags_bright_ground_points():
    # Two "paint lines" at y = -2m and y = +2m (rows), dim background elsewhere.
    # Step (0.05m) finer than RESOLUTION (0.2m) so multiple points land in
    # each grid cell -> clears the MARKING_MIN_HITS trust threshold.
    xs = np.tile(np.arange(-15.0, 15.0, 0.05), 3)
    n_per_line = len(np.arange(-15.0, 15.0, 0.05))
    ys = np.concatenate([np.full(n_per_line, -2.0),
                         np.full(n_per_line, 0.0),
                         np.full(n_per_line, 2.0)])
    zs = np.zeros_like(xs)
    intensity = np.concatenate([np.full(n_per_line, 200.0),  # bright paint line
                                np.full(n_per_line, 10.0),   # dim asphalt
                                np.full(n_per_line, 200.0)])  # bright paint line
    points = np.stack([xs, ys, zs, intensity], axis=1)

    evidence = ls.intensity_lane_evidence(points)

    gr_bright1 = int((-2.0 - ORIGIN_Y) / RESOLUTION)
    gr_dim     = int((0.0 - ORIGIN_Y) / RESOLUTION)
    gr_bright2 = int((2.0 - ORIGIN_Y) / RESOLUTION)

    assert evidence[gr_bright1, VEHICLE_COL] > 0.5
    assert evidence[gr_bright2, VEHICLE_COL] > 0.5
    assert evidence[gr_dim, VEHICLE_COL] < 0.5


def test_intensity_lane_evidence_empty_input():
    evidence = ls.intensity_lane_evidence(np.zeros((0, 4)))
    assert evidence.shape == (GRID_SIZE, GRID_SIZE)
    assert not evidence.any()

    evidence_none = ls.intensity_lane_evidence(None)
    assert not evidence_none.any()


def test_segment_lanes_splits_three_clean_lanes():
    mask, lane_row_ranges = make_three_lane_drivable_mask()
    marking = marking_lines_at_gaps(lane_row_ranges)

    lane_id_grid, confidence_grid = ls.segment_lanes(mask, marking)
    total, ego_idx, width_m = ls.count_and_locate_ego(lane_id_grid)

    assert total == 3
    assert ego_idx == 1  # ego sits in the middle lane
    assert width_m == pytest.approx(15 * RESOLUTION)
    assert confidence_grid[VEHICLE_ROW, VEHICLE_COL] == ls.CONF_ASSIGNED


def test_segment_lanes_robust_to_false_negative_gap_in_drivable_mask():
    """Drivable-grid false negative: a chunk of the ego's own lane is
    incorrectly marked non-drivable (e.g. a shadow). The lane should still
    be found and correctly counted since most of the lane region survives."""
    mask, lane_row_ranges = make_three_lane_drivable_mask()
    marking = marking_lines_at_gaps(lane_row_ranges)

    # Punch a hole in the ego lane away from the ego's own column.
    mask[lane_row_ranges[1][0]:lane_row_ranges[1][1], 40:60] = False

    lane_id_grid, confidence_grid = ls.segment_lanes(mask, marking)
    total, ego_idx, width_m = ls.count_and_locate_ego(lane_id_grid)

    assert total == 3
    assert ego_idx == 1
    assert confidence_grid[VEHICLE_ROW, VEHICLE_COL] == ls.CONF_ASSIGNED


def test_segment_lanes_robust_to_false_positive_blob_outside_road():
    """Drivable-grid false positive: an isolated drivable blob far from the
    road (e.g. misclassified sidewalk) must not get counted as a 4th lane,
    since it's not connected to the ego's lane component."""
    mask, lane_row_ranges = make_three_lane_drivable_mask()
    marking = marking_lines_at_gaps(lane_row_ranges)

    # Isolated blob, disconnected from the road (surrounded by non-drivable).
    mask[250:260, 250:260] = True

    lane_id_grid, confidence_grid = ls.segment_lanes(mask, marking)
    total, ego_idx, width_m = ls.count_and_locate_ego(lane_id_grid)

    assert total == 3
    assert ego_idx == 1
    # The blob is drivable but disconnected from any ego-column lane -> unassigned.
    assert lane_id_grid[255, 255] == -1
    assert confidence_grid[255, 255] == ls.CONF_UNASSIGNED


def test_segment_lanes_no_drivable_area():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    marking = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

    lane_id_grid, confidence_grid = ls.segment_lanes(mask, marking)
    total, ego_idx, width_m = ls.count_and_locate_ego(lane_id_grid)

    assert total == 0
    assert ego_idx == -1
    assert width_m == 0.0
    assert not confidence_grid.any()


def test_segment_lanes_single_lane_no_markings_detected():
    """If no marking evidence is found (e.g. worn-out paint) and the drivable
    strip is contiguous (no physical gap), it stays one lane rather than
    being split incorrectly."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[VEHICLE_ROW - 22:VEHICLE_ROW + 23, :] = True  # one wide contiguous strip
    marking = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)  # no markings found

    lane_id_grid, confidence_grid = ls.segment_lanes(mask, marking)
    total, ego_idx, width_m = ls.count_and_locate_ego(lane_id_grid)

    assert total == 1
    assert ego_idx == 0


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
