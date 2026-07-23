#!/usr/bin/env python3
"""
lane_barrier_fitting.py — bridge gaps in marking evidence so a real,
spatially-coherent line still cuts the road even where it wasn't directly
observed strongly enough.

Confirmed live: a bumper-mounted camera has a geometric blind spot for the
ground immediately next to the vehicle, and a thin painted line only fills
a fraction of a 0.2m grid cell (more so at range), so a real line's
detected strength is often weak exactly where lane_segmentation.segment_lanes
needs it most -- at the ego's own column. segment_lanes only cuts where
marking_evidence directly crosses MARKING_THRESHOLD, so a real line with a
weak/blind patch right at the ego's column never actually splits the road
there, even when it's confidently detected a few meters away in the same
grid.

This module extends road_corridor.py's scan-line-walk philosophy (a
smoothly-tracked float center, jump-clamped so noise can't yank it
sideways, walked outward in both directions from a seed) to the marking
evidence grid specifically. Two differences from road_corridor.py's walk:

  1. A marking barrier has no natural seed at the ego's own column -- that's
     exactly the blind/weak region -- so the seed has to come from wherever
     confident evidence actually exists, and the walk must *coast* through
     gaps (extrapolate using the recent trend) rather than stop the moment
     evidence disappears, which is what road_corridor.py's walk does and
     would just reproduce this bug if reused as-is. A coasted stretch is
     only trusted if the line is later reconfirmed by real evidence again --
     a trailing coast that never reconnects (the line simply ended, e.g. at
     an intersection) is trimmed away rather than left as a speculative tail
     extending max_gap_columns past the last real observation.

  2. A single column can have more than one real line at different rows at
     once (e.g. a left and a right lane divider both visible at the same
     distance down the road) -- road_corridor.py's single drivable/not-
     drivable value per cell has no such ambiguity, but marking_evidence
     does, so peak-finding here returns every sufficiently-separated local
     maximum per column, not just the single strongest.

Pure numpy, no rclpy/cv2 -- unit-testable standalone, same style as
road_corridor.py and lane_segmentation.py.
"""

import numpy as np

# Higher than lane_segmentation.MARKING_THRESHOLD (0.2, the "good enough to
# cut" bar): these cells anchor and steer an extrapolated line, so a weak or
# noisy cell shouldn't be trusted to seed or redirect one -- only a cell
# clearly at or above the strength a real, closely-observed line produces.
CONFIDENT_THRESHOLD = 0.35

# A tracked line needs at least this many *directly confirmed* (non-coasted)
# columns before it's trusted enough to be rasterized into the output --
# filters an isolated stray bright cell (noise) from being treated as a
# real line. 4 columns = 0.8m, short enough not to reject a real line seen
# only briefly, long enough that one-off noise can't seed a phantom cut.
MIN_CONFIRMED_COLUMNS = 4

# How far a line can be coasted (extrapolated, no confident evidence) before
# that direction gives up looking for a reconnection. Chosen well above the
# blind-spot gap width confirmed live (~30 cells / 6m) so the real gap is
# bridgeable, while still refusing to splice together two genuinely
# unrelated lines separated by a large unmapped stretch -- mirrors
# road_corridor.py's MAX_SIDE_REACH (a reach cap distinct from the per-step
# jump clamp below). Note a coast is only kept in the output if it actually
# reconnects within this budget (see _trim_trailing_coast) -- this constant
# bounds how far to *look* for a reconnection, not how far to speculate
# without one.
MAX_GAP_COLUMNS = 40

# Reused from road_corridor.py's defaults for the same role: how far a
# single step's found row may move from the tracked center (max_row_jump),
# and how wide a window to search for the nearest confident peak
# (search_radius).
DEFAULT_MAX_ROW_JUMP = 3
DEFAULT_SEARCH_RADIUS = 5

# Two candidate peaks in the same column closer than this (in rows) are
# treated as the same physical line (non-max suppression), not two lines --
# a real painted line is only ~1 cell wide as detected, so 5 cells (1m) is
# comfortably wider than one line's own footprint while still letting two
# genuinely close-together dividers (e.g. a turn-lane pair) register
# separately.
DEFAULT_MIN_PEAK_SEPARATION = 5


def _find_confident_peaks(col_evidence, drivable_col, confident_threshold, min_peak_separation):
    """col_evidence/drivable_col: 1-D arrays for one grid column.

    Returns a row-sorted list of local-maximum rows that clear
    confident_threshold, picked strongest-first with non-max suppression
    (min_peak_separation) so multiple picks aren't just adjacent cells of
    the same physical line.
    """
    candidates = np.flatnonzero(drivable_col & (col_evidence >= confident_threshold))
    if candidates.size == 0:
        return []
    order = candidates[np.argsort(-col_evidence[candidates])]
    picked = []
    for r in order:
        r = int(r)
        if all(abs(r - p) >= min_peak_separation for p in picked):
            picked.append(r)
    picked.sort()
    return picked


def _confident_peaks(marking_evidence, drivable_mask, confident_threshold, min_peak_separation):
    """Returns a length-W list of lists: for each column, every
    sufficiently-separated local-maximum row that clears confident_threshold
    (possibly empty, possibly more than one -- see module docstring)."""
    h, w = marking_evidence.shape
    return [
        _find_confident_peaks(marking_evidence[:, c], drivable_mask[:, c],
                               confident_threshold, min_peak_separation)
        for c in range(w)
    ]


def _nearest_unclaimed_peak(peaks_at_col, target, search_radius, claimed, col):
    """Nearest row in peaks_at_col to target (within search_radius) whose
    (col, row) isn't already in claimed. None if none qualify."""
    best, best_dist = None, None
    for r in peaks_at_col:
        if (col, r) in claimed:
            continue
        d = abs(r - target)
        if d <= search_radius and (best_dist is None or d < best_dist):
            best, best_dist = r, d
    return best


def _walk_evidence(peaks, claimed, seed_col, seed_row, direction,
                    max_row_jump, search_radius, max_gap_columns, grid_size):
    """Walks outward from (seed_col, seed_row) in one direction (+1 or -1),
    tracking a float center and a short recent slope (row-change per
    column, from the last confirmed step) -- see module docstring for the
    coast/reconnect philosophy.

    At each step: predict next_center = center + slope (so a resumed line
    is looked for where curvature says it should be, not just where it was
    last seen). If an unclaimed confident peak exists within search_radius
    of that prediction, claim it, snap to it (clamped to max_row_jump from
    the current center, so one noisy peak can't yank the walk), update the
    slope from the actual move, and mark the column confirmed. Otherwise
    coast: extrapolate the center by the slope and mark the column
    unconfirmed. Gives up once the gap streak exceeds max_gap_columns.

    Returns an ordered list of (col, row, confirmed_bool), outward from
    (but not including) the seed column -- the caller trims any trailing
    (never-reconnected) coast and adds the seed separately.
    """
    entries = []
    center = float(seed_row)
    slope = 0.0
    gap = 0
    c = seed_col
    while True:
        c += direction
        if not (0 <= c < grid_size):
            break
        predicted = center + slope
        raw_found = _nearest_unclaimed_peak(peaks[c], predicted, search_radius, claimed, c)

        if raw_found is not None:
            claimed.add((c, raw_found))
            found_row = raw_found
            if found_row > center + max_row_jump:
                found_row = center + max_row_jump
            elif found_row < center - max_row_jump:
                found_row = center - max_row_jump
            new_center = float(found_row)
            slope = new_center - center
            center = new_center
            gap = 0
            entries.append((c, int(round(center)), True))
        else:
            gap += 1
            if gap > max_gap_columns:
                break
            center = min(max(center + slope, 0.0), grid_size - 1.0)
            entries.append((c, int(round(center)), False))

    return entries


def _trim_trailing_coast(entries):
    """Drops any run of unconfirmed (coasted) entries at the end of the
    list -- a coast that never reconnected to real evidence again was
    speculation into the void (e.g. the line simply ended), not a bridged
    gap, and shouldn't be rasterized as if it were observed."""
    last_confirmed = -1
    for i, (_, _, confirmed) in enumerate(entries):
        if confirmed:
            last_confirmed = i
    return entries[:last_confirmed + 1]


def extract_barrier_lines(marking_evidence, drivable_mask,
                           confident_threshold=CONFIDENT_THRESHOLD,
                           min_confirmed_columns=MIN_CONFIRMED_COLUMNS,
                           max_gap_columns=MAX_GAP_COLUMNS,
                           max_row_jump=DEFAULT_MAX_ROW_JUMP,
                           search_radius=DEFAULT_SEARCH_RADIUS,
                           min_peak_separation=DEFAULT_MIN_PEAK_SEPARATION):
    """marking_evidence: (H, W) float32 in [0, 1]. drivable_mask: (H, W) bool
    (pass the already-corridor-cleaned mask, not the raw noisy one).

    Finds one or more independent marking lines by seeding a bidirectional
    walk from every sufficiently-confident, not-yet-claimed evidence peak,
    coasting each through gaps and trimming any trailing coast that never
    reconnects (see _walk_evidence / _trim_trailing_coast), and keeping only
    lines with enough directly-confirmed columns to be trusted. Naturally
    supports multiple simultaneous lines (different rows, possibly the same
    columns -- e.g. a left and right divider both visible at once), since
    peaks are found per-column as a list, not a single best row, and claims
    are tracked per (row, col) point rather than per column.

    Returns an (H, W) bool barrier_mask suitable for lane_segmentation.
    segment_lanes's barrier_mask parameter.
    """
    h, w = marking_evidence.shape
    peaks = _confident_peaks(marking_evidence, drivable_mask, confident_threshold, min_peak_separation)
    claimed = set()
    barrier_mask = np.zeros((h, w), dtype=bool)

    for c in range(w):
        for seed_row in peaks[c]:
            if (c, seed_row) in claimed:
                continue
            claimed.add((c, seed_row))

            left = _trim_trailing_coast(_walk_evidence(
                peaks, claimed, c, seed_row, -1, max_row_jump, search_radius, max_gap_columns, w))
            right = _trim_trailing_coast(_walk_evidence(
                peaks, claimed, c, seed_row, +1, max_row_jump, search_radius, max_gap_columns, w))

            confirmed_count = 1 + sum(1 for _, _, confirmed in left if confirmed) \
                                 + sum(1 for _, _, confirmed in right if confirmed)
            if confirmed_count >= min_confirmed_columns:
                barrier_mask[seed_row, c] = True
                for col, row, _ in left:
                    barrier_mask[row, col] = True
                for col, row, _ in right:
                    barrier_mask[row, col] = True

    return barrier_mask
