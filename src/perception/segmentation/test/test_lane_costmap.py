#!/usr/bin/env python3
"""
test_lane_costmap.py — tests for lane_costmap.py.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu
"""

import numpy as np

from segmentation.lane_costmap import (
    seeded_connected_mask, RollingMajorityFilter, distance_cost_grid,
)

H, W = 40, 40
EGO_ROW, EGO_COL = 20, 20


def _grid():
    return np.zeros((H, W), dtype=bool)


# ── seeded_connected_mask ────────────────────────────────────────────────

def test_seeded_connected_mask_keeps_component_touching_seed():
    mask = _grid()
    mask[EGO_ROW - 2:EGO_ROW + 3, EGO_COL - 1:EGO_COL + 2] = True  # a blob around ego

    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL)

    assert np.array_equal(out, mask)


def test_seeded_connected_mask_drops_disconnected_island():
    mask = _grid()
    mask[EGO_ROW - 2:EGO_ROW + 3, EGO_COL - 1:EGO_COL + 2] = True  # ego's real lane
    mask[5, 5] = True  # a totally disconnected noise speck elsewhere

    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL)

    assert out[EGO_ROW, EGO_COL]
    assert not out[5, 5]
    assert out.sum() == mask.sum() - 1


def test_seeded_connected_mask_drops_a_disconnected_other_lane():
    """A separate lane's blob, not touching ours, must be dropped even
    though it's a large, real, coherent region -- connectivity to the
    vehicle is what matters, not size or shape."""
    mask = _grid()
    mask[EGO_ROW - 2:EGO_ROW + 3, EGO_COL - 1:EGO_COL + 2] = True   # our lane
    mask[EGO_ROW - 2:EGO_ROW + 3, EGO_COL + 10:EGO_COL + 13] = True  # another lane, unconnected

    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL)

    assert out[EGO_ROW, EGO_COL]
    assert not out[EGO_ROW, EGO_COL + 11]


def test_seeded_connected_mask_searches_nearby_when_seed_cell_empty():
    mask = _grid()
    mask[EGO_ROW + 3, EGO_COL] = True  # a real detection just ahead, not exactly at ego

    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL, seed_search_radius=5)

    assert out[EGO_ROW + 3, EGO_COL]


def test_seeded_connected_mask_returns_empty_when_nothing_within_radius():
    mask = _grid()
    mask[EGO_ROW + 15, EGO_COL] = True  # far outside the search radius

    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL, seed_search_radius=5)

    assert not out.any()


def test_seeded_connected_mask_empty_input_returns_empty():
    mask = _grid()
    out = seeded_connected_mask(mask, EGO_ROW, EGO_COL)
    assert not out.any()


# ── RollingMajorityFilter ────────────────────────────────────────────────

def test_rolling_majority_filter_single_bad_frame_does_not_tip_result():
    f = RollingMajorityFilter(window=5, min_votes=3)
    steady = _grid()
    steady[EGO_ROW, EGO_COL] = True
    bleed = _grid()
    bleed[EGO_ROW, EGO_COL] = True
    bleed[EGO_ROW, EGO_COL + 10] = True  # one frame's worth of leaked adjacent lane

    for _ in range(3):
        out = f.update(steady)
    out = f.update(bleed)

    assert out[EGO_ROW, EGO_COL]
    assert not out[EGO_ROW, EGO_COL + 10]  # single-frame leak must not appear


def test_rolling_majority_filter_sustained_change_does_eventually_win():
    f = RollingMajorityFilter(window=5, min_votes=3)
    steady = _grid()
    steady[EGO_ROW, EGO_COL] = True
    changed = _grid()
    changed[EGO_ROW, EGO_COL + 10] = True

    for _ in range(3):
        f.update(steady)
    out = None
    for _ in range(5):
        out = f.update(changed)  # a real, sustained change over many frames

    assert out[EGO_ROW, EGO_COL + 10]


def test_rolling_majority_filter_stale_frame_ages_out_of_fixed_window():
    """A cell that was True for a while but then stops must go False again
    once enough fresh False frames have pushed it out of the window --
    unlike an OR-forever history, nothing here lingers indefinitely."""
    f = RollingMajorityFilter(window=5, min_votes=3)
    on = _grid()
    on[EGO_ROW, EGO_COL] = True
    off = _grid()

    for _ in range(5):
        f.update(on)
    out = None
    for _ in range(5):
        out = f.update(off)

    assert not out[EGO_ROW, EGO_COL]


def test_rolling_majority_filter_requires_min_votes_not_just_any_true():
    f = RollingMajorityFilter(window=5, min_votes=3)
    on = _grid()
    on[EGO_ROW, EGO_COL] = True
    off = _grid()

    f.update(on)
    f.update(off)
    out = f.update(off)  # only 1 of 3 frames so far was True

    assert not out[EGO_ROW, EGO_COL]


# ── distance_cost_grid ───────────────────────────────────────────────────

def test_distance_cost_grid_zero_at_deep_center():
    mask = _grid()
    mask[5:35, 15:25] = True  # a wide, deep region

    cost = distance_cost_grid(mask, max_dist_cells=4)

    assert cost[20, 20] == 0  # far from every edge


def test_distance_cost_grid_high_near_boundary():
    mask = _grid()
    mask[5:35, 15:25] = True

    cost = distance_cost_grid(mask, max_dist_cells=4)

    assert cost[5, 20] > 50  # right at the region's own edge row


def test_distance_cost_grid_max_outside_mask():
    mask = _grid()
    mask[5:35, 15:25] = True

    cost = distance_cost_grid(mask, max_dist_cells=4)

    assert cost[0, 0] == 100
    assert cost[39, 39] == 100


def test_distance_cost_grid_empty_mask_is_all_max_cost():
    mask = _grid()
    cost = distance_cost_grid(mask, max_dist_cells=4)
    assert (cost == 100).all()


def test_distance_cost_grid_narrower_region_never_reaches_zero():
    """A region narrower than 2*max_dist_cells can't contain any point
    that's max_dist_cells from every edge -- cost should bottom out above
    0 everywhere, not falsely claim a safe center that isn't there."""
    mask = _grid()
    mask[10:30, 19:21] = True  # only 2 cells wide

    cost = distance_cost_grid(mask, max_dist_cells=8)

    assert cost[20, 19] > 0
    assert cost[20, 20] > 0
