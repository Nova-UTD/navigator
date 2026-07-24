#!/usr/bin/env python3
"""
road_corridor.py — clean, geometry-aware road extent from a noisy drivable
mask.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

/grid/drivable/segmented (perception_drivable_grid_node.py's output) is a
per-cell classification and can be choppy — a misclassified sidewalk patch,
a driveway, or a noisy bridge between the road and an adjacent surface can
all read as "drivable" alongside the real road. Feeding that directly into
lane_segmentation.segment_lanes lets an unrelated blob that happens to
cross the ego's column get counted as its own "lane".

Three independent problems, three mitigations:

1. Disconnected/noise-bridged blobs (e.g. a sidewalk patch that isn't
   really joined to the road, or is only joined via one noisy cell).
   Fixed by a per-axis scan-line tracker that starts from the ego's own
   (known-good) position and, step by step, keeps only the single
   contiguous span containing the tracked center, capped to a max
   half-width -- not the whole connected blob.

2. Sustained misclassification: PSPNet reading an entire adjacent sidewalk
   as "road" for real, not as noise -- confirmed by live testing (a scene
   where the camera looked mostly at a sidewalk/building, and PSPNet
   labeled nearly all of it road-colored). This is visually and even
   loosely geometrically indistinguishable from real road by classification
   alone: both are flat, paved, grayish surfaces. What *does* distinguish
   them is the physical curb between them -- a real height step LiDAR can
   see even when the camera/classifier can't tell the difference. When a
   height grid is supplied, the tracker refuses to cross a height jump
   greater than max_height_jump, treating it as a curb wall exactly like a
   drivable/non-drivable boundary.

3. The road isn't necessarily ahead of or behind the vehicle -- confirmed
   live: a vehicle stopped facing a building/plaza with the actual road
   only visible to its side. Marking detection already fuses all 4
   cameras into one persistent grid regardless of heading, but the
   original version of this tracker only walked along the vehicle's own
   forward axis (columns), so it could never reach a road that wasn't
   roughly in front of or behind it. extract_ego_road_corridor now runs
   the same scan-line walk along BOTH axes -- forward/backward (columns)
   and side-to-side (rows) -- from the ego position and takes the union,
   so a road to the side is found too. Each axis still independently
   applies its own perpendicular width cap and curb check, so the
   noisy-bridge protection from (1) holds in both directions.

Pure numpy, no rclpy/cv2 — unit-testable standalone, same style as
lane_segmentation.py.
"""

import numpy as np

# A real curb is typically several cm to ~15cm; small enough that camera
# lighting won't fake it, large enough that LiDAR ground-height noise
# (a couple cm) won't false-trigger it. Starting point -- expect live
# tuning against real curb heights, same as other thresholds tonight.
DEFAULT_MAX_HEIGHT_JUMP = 0.06  # meters

# Generous cap on how far a kept span can extend from the tracked center in
# either direction (6m each way = 12m total at 0.2m/cell -- wide enough for
# a real multi-lane road, narrow enough to exclude an unrelated surface
# that happens to fuse into the same contiguous run at one noisy step).
MAX_HALF_WIDTH = 30

# The perpendicular width cap only bounds how wide a kept span is *at each
# step* -- it does nothing to stop a walk from taking many steps through a
# narrow (even single-cell) noisy bridge, since each individual step still
# looks locally valid. The forward/backward walk legitimately needs to
# travel far (following a route into the distance), so it stays unbounded.
# The side-to-side walk exists only to find a road that's near the vehicle
# but not aligned with its heading -- it doesn't need unlimited reach, and
# capping it is what actually stops a many-step walk through a narrow
# bridge into an unrelated, far-off surface.
MAX_SIDE_REACH = 60  # cells (12m) each direction


def _height_ok(height_col, r1, r2, max_height_jump):
    """True if there's no disqualifying height jump between adjacent rows
    r1, r2 in height_col (NaN = unobserved, never blocks -- no LiDAR data
    there is not evidence of a curb, just missing information)."""
    if max_height_jump is None or height_col is None:
        return True
    h1, h2 = height_col[r1], height_col[r2]
    if np.isnan(h1) or np.isnan(h2):
        return True
    return abs(h1 - h2) <= max_height_jump


def _contiguous_run_containing(column, row, height_col=None, max_height_jump=None):
    """column: 1-D bool array. row: index believed to be True in column.

    Returns (start, end) of the maximal contiguous True run containing
    row (end exclusive), or None if column[row] is False. If height_col is
    given, the run also stops at any adjacent-row height jump greater than
    max_height_jump (a curb), even if both cells are otherwise "drivable".
    """
    if not column[row]:
        return None
    start = row
    while start > 0 and column[start - 1] and _height_ok(height_col, start - 1, start, max_height_jump):
        start -= 1
    end = row + 1
    n = len(column)
    while end < n and column[end] and _height_ok(height_col, end - 1, end, max_height_jump):
        end += 1
    return start, end


def _nearest_true_row(column, seed_row, search_radius):
    """Nearest row to seed_row (within search_radius) where column is True,
    preferring seed_row itself, then expanding outward. None if none found."""
    n = len(column)
    if 0 <= seed_row < n and column[seed_row]:
        return seed_row
    for d in range(1, search_radius + 1):
        for r in (seed_row - d, seed_row + d):
            if 0 <= r < n and column[r]:
                return r
    return None


def _height_ok_2d(height_grid, r1, c1, r2, c2, max_height_jump):
    """Same rule as _height_ok, but for two arbitrary (row, col) points
    instead of two rows within one column -- used to check continuity
    *between* consecutive outward steps (which may be a row-to-row move,
    for the side-to-side walk), not just within one step's perpendicular
    span."""
    if max_height_jump is None or height_grid is None:
        return True
    h1, h2 = height_grid[r1, c1], height_grid[r2, c2]
    if np.isnan(h1) or np.isnan(h2):
        return True
    return abs(h1 - h2) <= max_height_jump


def _walk_columns(drivable_mask, ego_row, ego_col,
                   search_radius, max_row_jump, max_half_width,
                   height_grid, max_height_jump, max_outward_steps=None):
    """Single-axis scan-line tracker: walks outward from ego_col along
    columns, keeping at each column only the contiguous row-span containing
    a smoothly-tracked center. See module docstring, mitigation (1)/(2).

    Height continuity is checked in two places: within each column's
    perpendicular span (via capped_run) AND between consecutive outward
    steps (the found_row-to-found_row move column-to-column) -- the latter
    matters when this function is run on a transposed grid for the
    side-to-side walk (mitigation (3)), where the outward-stepping
    direction is itself the one a curb would run across.

    max_outward_steps (None = unbounded) caps how many columns outward
    from ego_col this walk will take in each direction -- see
    MAX_SIDE_REACH for why this matters for the side-to-side walk
    specifically.
    """
    h, w = drivable_mask.shape
    clean = np.zeros((h, w), dtype=bool)

    if not (0 <= ego_row < h and 0 <= ego_col < w) or not drivable_mask[ego_row, ego_col]:
        return clean  # ego itself isn't on a drivable cell -- nothing to seed from

    def height_col_at(c):
        return None if height_grid is None else height_grid[:, c]

    def capped_run(column, row, height_col):
        run = _contiguous_run_containing(column, row, height_col, max_height_jump)
        if run is None:
            return None
        start, end = run
        return max(start, row - max_half_width), min(end, row + max_half_width + 1)

    start, end = capped_run(drivable_mask[:, ego_col], ego_row, height_col_at(ego_col))
    clean[start:end, ego_col] = True
    center = (start + end - 1) / 2.0

    for direction in (1, -1):
        c = ego_col
        cur_center = center
        prev_row, prev_col = ego_row, ego_col
        while True:
            c += direction
            if not (0 <= c < w):
                break
            if max_outward_steps is not None and abs(c - ego_col) > max_outward_steps:
                break
            seed_row = int(round(cur_center))
            found_row = _nearest_true_row(drivable_mask[:, c], seed_row, search_radius)
            if found_row is None:
                break  # edge of what's been observed/confirmed -- stop here
            # Clamp the jump so one noisy column can't yank the corridor
            # sideways; curves are still followed, just gradually.
            if found_row > seed_row + max_row_jump:
                found_row = seed_row + max_row_jump
            elif found_row < seed_row - max_row_jump:
                found_row = seed_row - max_row_jump
            if not drivable_mask[found_row, c]:
                # Jump clamp landed off the actual drivable cell; nothing
                # trustworthy to extend to at this column.
                break
            if not _height_ok_2d(height_grid, prev_row, prev_col, found_row, c, max_height_jump):
                break  # a curb runs across the outward-stepping direction itself
            start, end = capped_run(drivable_mask[:, c], found_row, height_col_at(c))
            clean[start:end, c] = True
            cur_center = (start + end - 1) / 2.0
            prev_row, prev_col = found_row, c

    return clean


def extract_ego_road_corridor(drivable_mask, ego_row, ego_col,
                               search_radius=5, max_row_jump=3,
                               max_half_width=MAX_HALF_WIDTH,
                               height_grid=None,
                               max_height_jump=DEFAULT_MAX_HEIGHT_JUMP,
                               max_side_reach=MAX_SIDE_REACH):
    """drivable_mask: (H, W) bool. ego_row/ego_col: the vehicle's own cell,
    assumed drivable by construction (it's sitting there).
    height_grid: optional (H, W) float, LiDAR ground height per cell (NaN
    where unobserved). When given, the corridor also refuses to cross a
    height jump > max_height_jump between adjacent cells -- a curb, even
    if the camera classifies both sides as road.

    Returns an (H, W) bool grid containing only the tracked road corridor:
    the union of a scan-line walk along columns (forward/backward from the
    vehicle, unbounded reach -- it's meant to see as far down the road as
    confirmed) and one along rows (side-to-side, reach capped by
    max_side_reach -- it exists to find a road near the vehicle that isn't
    aligned with its heading, not to search arbitrarily far sideways
    through a narrow bridge). Each keeps only the contiguous span
    containing a smoothly-tracked center, capped to max_half_width, so a
    disconnected or noise-bridged region at an unrelated offset is still
    excluded in whichever axis would otherwise have leaked into it. See
    module docstring.
    """
    along_columns = _walk_columns(
        drivable_mask, ego_row, ego_col,
        search_radius, max_row_jump, max_half_width, height_grid, max_height_jump)

    height_grid_t = None if height_grid is None else height_grid.T
    along_rows_t = _walk_columns(
        drivable_mask.T, ego_col, ego_row,
        search_radius, max_row_jump, max_half_width, height_grid_t, max_height_jump,
        max_outward_steps=max_side_reach)
    along_rows = along_rows_t.T

    return along_columns | along_rows
