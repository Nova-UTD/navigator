"""Unit tests for road_corridor.py — pure numpy, no ROS required.

Run with: python3 -m pytest src/perception/segmentation/test/test_road_corridor.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import road_corridor as rc

GRID_SIZE = 300
EGO_ROW = 150
EGO_COL = 100


def test_straight_clean_road_matches_exactly():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :] = True  # one clean 20-cell-wide straight road

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert np.array_equal(clean, mask)


def test_excludes_disconnected_parallel_sidewalk():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :] = True   # road
    mask[200:210, :] = True   # disconnected "sidewalk" strip, far away

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert clean[140:160, :].all()
    assert not clean[200:210, :].any()


def test_excludes_sidewalk_connected_via_noisy_bridge():
    """The exact reported bug pattern: a sidewalk strip that IS technically
    connected to the road via a single noisy misclassified column (so
    scipy.ndimage.label would treat it as one component), but is not a
    real lane -- the corridor tracker should stay on the road because the
    bridge is far outside the tracked center's search radius at that
    column, not because of disconnection."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :] = True   # road
    mask[200:210, :] = True   # sidewalk strip
    mask[160:200, 150] = True  # noisy bridge connecting them at column 150

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert clean[140:160, :].all()
    assert not clean[200:210, :].any()


def test_follows_a_smooth_curve():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    # Road drifts from row 150 at col 100 up to row 200 by col 250,
    # a gentle curve well within the per-column jump budget.
    for col in range(GRID_SIZE):
        if col < 100:
            center = 150
        else:
            center = 150 + min(50, (col - 100) // 3)
        mask[center - 10:center + 10, col] = True

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    # Should have tracked all the way to the far end of the curve.
    assert clean[:, GRID_SIZE - 1].any()
    far_rows = np.flatnonzero(clean[:, GRID_SIZE - 1])
    assert 190 <= far_rows.mean() <= 210


def test_stops_at_unmapped_gap_instead_of_jumping():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :180] = True  # road ends at column 180 (unmapped beyond)
    mask[140:160, 220:] = True  # road resumes far past a real gap

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert clean[140:160, 170:180].all()
    assert not clean[:, 180:220].any()
    assert not clean[:, 220:].any()  # never reconnects across a real gap


def test_height_jump_excludes_directly_adjacent_misclassified_sidewalk():
    """The confirmed live failure mode: PSPNet reading an entire adjacent
    sidewalk as "road" for real (not noise) -- directly touching the real
    road with no gap or bridge needed to demonstrate the bug. A real curb
    step in the height grid should stop the corridor at the true road edge
    even though drivable_mask says both sides are drivable."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :] = True   # road
    mask[160:180, :] = True   # sidewalk, directly adjacent, misclassified drivable

    height = np.full((GRID_SIZE, GRID_SIZE), np.nan, dtype=np.float32)
    height[140:160, :] = 0.0    # road surface height
    height[160:180, :] = 0.15   # curb step up to sidewalk

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL, height_grid=height)

    assert clean[140:160, :].all()
    assert not clean[160:180, :].any()


def test_height_jump_ignored_without_lidar_data():
    """NaN (no LiDAR observation) must never itself be treated as a curb --
    missing information isn't evidence of one."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:180, :] = True  # one 40-cell-wide road, no real curb anywhere

    height = np.full((GRID_SIZE, GRID_SIZE), np.nan, dtype=np.float32)  # no LiDAR data at all

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL, height_grid=height)

    assert clean[140:180, :].all()


def test_finds_road_to_the_side_not_just_ahead_or_behind():
    """The reported gap: a vehicle stopped facing a building/plaza with the
    actual road only visible to its side (perpendicular to its own
    heading) -- the corridor must still find it, not just search along the
    vehicle's own forward/backward axis."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    # Road runs perpendicular to the vehicle's forward axis: a vertical
    # strip in COLUMN space (i.e. spans all rows at a fixed column range),
    # passing through the ego's own row/col so it's reachable, but nothing
    # exists ahead or behind the ego at its own row.
    mask[:, 90:110] = True

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    # Found the road running perpendicular to the ego's own column axis --
    # i.e. it extends well beyond the ego's own row in the ROW direction
    # near the ego's column (bounded by the side-walk's reach cap, not
    # unlimited -- see test_bidirectional_side_reach_is_capped).
    assert clean[EGO_ROW - 40, EGO_COL]
    assert clean[EGO_ROW + 40, EGO_COL]


def test_bidirectional_side_reach_is_capped():
    """Unlike the forward/backward walk (which needs to see arbitrarily far
    down a route), the side-to-side walk exists only to find a road near
    the vehicle that isn't aligned with its heading -- it must not follow
    a many-step, single-column-wide noisy bridge indefinitely into an
    unrelated far-off surface. The perpendicular width cap alone can't
    catch this (each individual step through the narrow bridge still
    looks locally valid); the reach cap on the side-to-side walk itself
    is what stops it."""
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[140:160, :] = True             # road, spans all columns
    mask[160:280, EGO_COL] = True       # long noisy bridge at the ego's own column
    mask[280:290, :] = True             # unrelated far-off surface

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert clean[140:160, :].all()
    assert not clean[280:290, :].any()


def test_ego_not_on_drivable_cell_returns_empty():
    mask = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
    mask[0:10, 0:10] = True  # drivable area exists, but not under the ego

    clean = rc.extract_ego_road_corridor(mask, EGO_ROW, EGO_COL)

    assert not clean.any()


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
