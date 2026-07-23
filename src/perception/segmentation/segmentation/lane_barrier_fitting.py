#!/usr/bin/env python3
"""
lane_barrier_fitting.py — once a real lane divider is confidently detected
anywhere, trust it for the length of the road corridor instead of only
cutting where evidence was directly observed above threshold.

Confirmed live: a bumper-mounted camera has a geometric blind spot for the
ground immediately next to the vehicle, and a thin painted line only fills
a fraction of a 0.2m grid cell (more so at range), so a real line's
detected strength is often weak exactly where lane_segmentation.segment_lanes
needs it most -- at the ego's own column. segment_lanes only cuts where
marking_evidence directly crosses MARKING_THRESHOLD, so a real line with a
weak/blind patch right at the ego's column never actually splits the road
there, even when it's confidently detected a few meters away in the same
grid.

An earlier version of this module only bridged gaps up to a fixed budget
and discarded any extrapolation that didn't reconnect to fresh confident
evidence again. That was too conservative and produced exactly the
flickering, inconsistent splits reported live: a real physical lane
divider is one continuous painted stripe for the length of a lane -- it
doesn't intermittently vanish and reappear within a single field of view,
so treating "no confident evidence this column" as reason to distrust an
already-established line was the wrong default. This version instead: once
a line clears a minimum-confirmed-columns noise filter, its tracked
direction is projected through the *entire* remaining corridor (to the
grid/corridor boundary), not capped at a modest gap budget -- real
evidence, when it reappears, still gets snapped to for accuracy, but its
absence is no longer treated as a reason to stop trusting the line.

This module extends road_corridor.py's scan-line-walk philosophy (a
smoothly-tracked float center, jump-clamped to follow curves without being
yanked by noise, walked outward in both directions from a seed) to the
marking evidence grid specifically. Two differences from road_corridor.py's
walk:

  1. A marking barrier has no natural seed at the ego's own column -- that's
     exactly the blind/weak region -- so the seed has to come from wherever
     confident evidence actually exists, and the walk projects outward
     through the whole corridor from there (see above), rather than
     stopping the moment evidence disappears the way road_corridor.py's
     walk does (which would just reproduce this bug if reused as-is).

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

# Matches lane_segmentation.MARKING_THRESHOLD. Live-tuned down from an
# initial 0.35 (meant to be a stricter bar than a bare cut, on the theory
# that cells anchoring/steering a line should be held to a higher standard)
# -- confirmed live that real accumulated evidence for an actually-visible
# line commonly tops out around 0.2-0.3, so 0.35 left too few columns
# confidently seedable. These cells aren't meaningfully more trustworthy
# than a bare cut, so there's no reason to demand more of them.
CONFIDENT_THRESHOLD = 0.2

# A tracked line needs at least this many *directly confirmed* columns
# before it's trusted enough to be projected through the corridor -- filters
# an isolated stray bright cell (noise) from being treated as a real line.
# 4 columns = 0.8m, short enough not to reject a real line seen only
# briefly, long enough that one-off noise can't seed a phantom cut.
MIN_CONFIRMED_COLUMNS = 4

# Reused from road_corridor.py's defaults for the same role: how far a
# single step's found row may move from the tracked center (max_row_jump),
# and how wide a window to search for the nearest confident peak to snap to
# when real evidence does reappear (search_radius).
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
                    max_row_jump, search_radius, grid_size):
    """Walks outward from (seed_col, seed_row) in one direction (+1 or -1)
    all the way to the grid boundary, tracking a float center and a short
    recent slope (row-change per column, from the last confirmed step) --
    see module docstring for why an already-established line is projected
    through the whole remaining extent rather than given up on when
    evidence is absent.

    At each step: predict next_center = center + slope (so the projection
    follows curvature already implied by the line's own recent trend, not
    just a flat hold). If an unclaimed confident peak exists within
    search_radius of that prediction, claim it, snap to it (clamped to
    max_row_jump from the current center, so one noisy peak can't yank the
    line), and update the slope from the actual move -- real evidence, when
    present, still wins over pure extrapolation. Otherwise the column is
    filled from the extrapolated center alone.

    Returns an ordered list of (col, row, confirmed_bool), outward from
    (but not including) the seed column -- confirmed_bool records whether
    that column snapped to real evidence, for the caller's noise-filter
    count, but every column (confirmed or not) is returned and meant to be
    rasterized once the line as a whole passes that filter.
    """
    entries = []
    center = float(seed_row)
    slope = 0.0
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
            entries.append((c, int(round(center)), True))
        else:
            center = min(max(center + slope, 0.0), grid_size - 1.0)
            entries.append((c, int(round(center)), False))

    return entries


def extract_barrier_lines(marking_evidence, drivable_mask,
                           confident_threshold=CONFIDENT_THRESHOLD,
                           min_confirmed_columns=MIN_CONFIRMED_COLUMNS,
                           max_row_jump=DEFAULT_MAX_ROW_JUMP,
                           search_radius=DEFAULT_SEARCH_RADIUS,
                           min_peak_separation=DEFAULT_MIN_PEAK_SEPARATION):
    """marking_evidence: (H, W) float32 in [0, 1]. drivable_mask: (H, W) bool
    (pass the already-corridor-cleaned mask, not the raw noisy one).

    Finds one or more independent marking lines by seeding a bidirectional
    walk from every sufficiently-confident, not-yet-claimed evidence peak,
    then projecting each through the entire corridor (see _walk_evidence),
    keeping only lines with enough directly-confirmed columns to be trusted
    as real rather than noise. Naturally supports multiple simultaneous
    lines (different rows, possibly the same columns -- e.g. a left and
    right divider both visible at once), since peaks are found per-column
    as a list, not a single best row, and claims are tracked per
    (row, col) point rather than per column.

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

            left = _walk_evidence(peaks, claimed, c, seed_row, -1, max_row_jump, search_radius, w)
            right = _walk_evidence(peaks, claimed, c, seed_row, +1, max_row_jump, search_radius, w)

            confirmed_count = 1 + sum(1 for _, _, confirmed in left if confirmed) \
                                 + sum(1 for _, _, confirmed in right if confirmed)
            if confirmed_count >= min_confirmed_columns:
                barrier_mask[seed_row, c] = True
                for col, row, _ in left:
                    barrier_mask[row, col] = True
                for col, row, _ in right:
                    barrier_mask[row, col] = True

    return barrier_mask
