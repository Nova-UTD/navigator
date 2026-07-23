"""Unit tests for lane_barrier_fitting.py — pure numpy, no ROS required.

Run with: python3 -m pytest src/perception/segmentation/test/test_lane_barrier_fitting.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import lane_barrier_fitting as lbf
from segmentation.lane_segmentation import segment_lanes, count_and_locate_ego
from segmentation.bev_geometry import GRID_SIZE, VEHICLE_ROW, VEHICLE_COL


def test_single_confident_run_projects_through_the_whole_corridor():
    """Core behavior: a real physical lane divider is one continuous
    stripe for the length of a lane -- once confidently detected anywhere,
    it must be trusted for the entire corridor, not just the columns it
    was directly observed at."""
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[140:160, :] = True
    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    evidence[150, 50:101] = 0.8  # only directly observed across 51 columns

    barrier = lbf.extract_barrier_lines(evidence, drivable)

    # Projected through the entire drivable width, not just where observed.
    assert barrier[150, :].all()


def test_a_large_gap_is_still_bridged():
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[140:160, :] = True
    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    # Two confirmed segments with a 150-column gap between them -- far
    # larger than the old conservative bridging budget ever allowed.
    evidence[150, 20:41] = 0.8
    evidence[150, 191:211] = 0.8

    barrier = lbf.extract_barrier_lines(evidence, drivable)

    assert barrier[150, 20:41].all()
    assert barrier[150, 191:211].all()
    assert barrier[150, 100]  # bridged through the gap's middle
    assert barrier[150, :].all()  # and projected through the whole corridor beyond both ends too


def test_isolated_short_blob_is_filtered_out():
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[140:160, :] = True
    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    # Only 2 confident columns -- below MIN_CONFIRMED_COLUMNS (4), should be
    # treated as noise, not a real line.
    evidence[150, 60:62] = 0.8

    barrier = lbf.extract_barrier_lines(evidence, drivable)

    assert not barrier.any()


def test_finds_two_separate_lines_independently():
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[100:220, :] = True
    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    evidence[130, 20:41] = 0.8  # first line
    evidence[200, 20:41] = 0.8  # second, independent line elsewhere on the road

    barrier = lbf.extract_barrier_lines(evidence, drivable)

    assert barrier[130, :].all()
    assert barrier[200, :].all()


def test_follows_a_gradually_curving_line():
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[100:220, :] = True
    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    # Row drifts by 1 every 5 columns -- well within max_row_jump (3) per step.
    for col in range(50, 200):
        row = 150 + (col - 50) // 5
        evidence[row, col] = 0.8

    barrier = lbf.extract_barrier_lines(evidence, drivable)

    far_rows = np.flatnonzero(barrier[:, 199])
    assert far_rows.size > 0
    expected_row = 150 + (199 - 50) // 5
    assert abs(int(far_rows[0]) - expected_row) <= 3


def test_barrier_mask_bridges_gap_at_ego_column_and_splits_correctly():
    """Core regression test: the confirmed live bug -- a real divider line
    with a blind-spot gap straddling the ego's own column must still
    produce a correctly-split lane count once run through segment_lanes
    with extract_barrier_lines's output."""
    drivable = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    drivable[125:170, :] = True  # wide road; ego (row 150) sits off the divider

    evidence = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
    divider_row = 145
    evidence[divider_row, :] = 0.8
    gap = slice(VEHICLE_COL - 15, VEHICLE_COL + 15)  # 30-cell blind-spot gap at the ego's column
    evidence[divider_row, gap] = 0.0

    # Without gap-bridging, the naive threshold path can't split at the ego's column.
    lane_id_grid_naive, _ = segment_lanes(drivable, evidence)
    naive_count, _, _ = count_and_locate_ego(lane_id_grid_naive)
    assert naive_count == 1

    barrier_mask = lbf.extract_barrier_lines(evidence, drivable)
    lane_id_grid, _ = segment_lanes(drivable, evidence, barrier_mask=barrier_mask)
    total_lane_count, ego_lane_index, _ = count_and_locate_ego(lane_id_grid)

    assert total_lane_count == 2
    assert ego_lane_index in (0, 1)


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
