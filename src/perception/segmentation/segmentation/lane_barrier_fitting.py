#!/usr/bin/env python3
"""
lane_barrier_fitting.py — detect candidate lane-divider lines and project
each through the entire road corridor via its tracked direction, instead of
only cutting where evidence was directly observed above threshold.

Confirmed live: a bumper-mounted camera has a geometric blind spot for the
ground immediately next to the vehicle, and a thin painted line only fills
a fraction of a 0.2m grid cell (more so at range), so a real line's
detected strength is often weak exactly where lane_segmentation.segment_lanes
needs it most -- at the ego's own column. A real physical lane divider is
one continuous painted stripe for the length of a lane -- it doesn't
intermittently vanish and reappear within a single field of view -- so
once a candidate clears a minimum-confirmed-columns noise filter, its
tracked direction is projected through the *entire* remaining corridor
(to the grid/corridor boundary), not capped at a gap budget. Real evidence,
when it reappears, still gets snapped to for accuracy.

This module only detects and geometrically extrapolates candidates for a
SINGLE tick -- it has no memory across publish cycles. Live testing found
that trusting a candidate purely from one tick's evidence is too easy for
noise to clear (a spurious run occasionally produces enough confirmed
columns to look real for one frame), so this module now returns structured
`CandidateLine` objects rather than a flattened boolean mask, and cross-tick
confirmation (does this candidate keep showing up, tick after tick?) plus
duplicate-line suppression (are two "different" candidates actually the
same physical line, too close together to both be real?) live in
`lane_line_tracker.py`, which decides which candidates actually get to cut
the road. Use `candidates_to_mask` to rasterize whatever candidates the
tracker has decided to trust into the boolean mask
`lane_segmentation.segment_lanes`'s `barrier_mask` parameter expects.

This module extends road_corridor.py's scan-line-walk philosophy (a
smoothly-tracked float center, jump-clamped to follow curves without being
yanked by noise, walked outward in both directions from a seed) to the
marking evidence grid specifically. Two differences from road_corridor.py's
walk:

  1. A marking barrier has no natural seed at the ego's own column -- that's
     exactly the blind/weak region -- so the seed has to come from wherever
     confident evidence actually exists, and the walk projects outward
     through the whole corridor from there, rather than stopping the moment
     evidence disappears the way road_corridor.py's walk does (which would
     just reproduce this bug if reused as-is).

  2. A single column can have more than one real line at different rows at
     once (e.g. a left and a right lane divider both visible at the same
     distance down the road) -- road_corridor.py's single drivable/not-
     drivable value per cell has no such ambiguity, but marking_evidence
     does, so peak-finding here returns every sufficiently-separated local
     maximum per column, not just the single strongest.

Pure numpy, no rclpy/cv2 -- unit-testable standalone, same style as
road_corridor.py and lane_segmentation.py.
"""

from dataclasses import dataclass

import numpy as np

# Matches lane_segmentation.MARKING_THRESHOLD. Live-tuned down from an
# initial 0.35 (meant to be a stricter bar than a bare cut, on the theory
# that cells anchoring/steering a line should be held to a higher standard)
# -- confirmed live that real accumulated evidence for an actually-visible
# line commonly tops out around 0.2-0.3, so 0.35 left too few columns
# confidently seedable. These cells aren't meaningfully more trustworthy
# than a bare cut, so there's no reason to demand more of them.
CONFIDENT_THRESHOLD = 0.2

# Live-tuned down from 20. Back when clearing this bar granted immediate
# full-corridor cutting trust, it needed to be a high bar (20 columns / 4m)
# to keep transient noise from single-handedly cutting the whole road --
# even so, live testing found some noise runs still occasionally cleared
# it. Now that lane_line_tracker.py requires a candidate to be independently
# re-detected over multiple ticks (and deduplicates near-identical
# candidates) before it can actually cut anything, this filter's job
# shrinks to a cheap admission check -- is this walk even worth handing to
# the tracker -- not "is this walk trustworthy enough to act on right now."
# 8 columns (1.6m) comfortably rejects single-cell noise while not
# delaying a real, partially-occluded line from ever entering the tracker.
MIN_CONFIRMED_COLUMNS = 8

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


@dataclass
class CandidateLine:
    """One candidate marking line detected in a single tick, spanning the
    full column range edge-to-edge (since a walk always reaches the grid
    boundary in both directions -- see module docstring).

    seed_row/seed_col: where this candidate was first found.
    row_by_col: (grid_size,) float32, this line's row estimate at every
      column -- always fully defined, no gaps.
    confirmed_by_col: (grid_size,) bool, True where that estimate came from
      real snapped evidence rather than pure extrapolation.
    confirmed_columns: cached confirmed_by_col.sum() -- the per-tick
      admission strength checked against min_confirmed_columns. Deliberately
      NOT used to grant cutting trust directly anymore; that's
      lane_line_tracker.py's job, based on cross-tick persistence.
    """
    seed_row: int
    seed_col: int
    row_by_col: np.ndarray
    confirmed_by_col: np.ndarray
    confirmed_columns: int


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

    Returns (cols, rows, confirmed) -- three same-length lists, outward
    from (but not including) the seed column. confirmed[i] records whether
    that column snapped to real evidence, for the caller's admission-count
    check; every column (confirmed or not) is returned and meant to be
    included in the candidate's geometry once accepted.
    """
    cols, rows, confirmed = [], [], []
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
            cols.append(c); rows.append(center); confirmed.append(True)
        else:
            center = min(max(center + slope, 0.0), grid_size - 1.0)
            cols.append(c); rows.append(center); confirmed.append(False)

    return cols, rows, confirmed


def extract_barrier_lines(marking_evidence, drivable_mask,
                           confident_threshold=CONFIDENT_THRESHOLD,
                           min_confirmed_columns=MIN_CONFIRMED_COLUMNS,
                           max_row_jump=DEFAULT_MAX_ROW_JUMP,
                           search_radius=DEFAULT_SEARCH_RADIUS,
                           min_peak_separation=DEFAULT_MIN_PEAK_SEPARATION):
    """marking_evidence: (H, W) float32 in [0, 1]. drivable_mask: (H, W) bool
    (pass the already-corridor-cleaned mask, not the raw noisy one).

    Finds candidate marking lines by seeding a bidirectional walk from every
    sufficiently-confident, not-yet-claimed evidence peak, then projecting
    each through the entire corridor (see _walk_evidence). Naturally
    supports multiple simultaneous lines (different rows, possibly the same
    columns -- e.g. a left and right divider both visible at once), since
    peaks are found per-column as a list, not a single best row, and claims
    are tracked per (row, col) point rather than per column.

    Returns a list of CandidateLine, one per walk whose confirmed_columns
    clears min_confirmed_columns -- a cheap per-tick admission filter only
    (see MIN_CONFIRMED_COLUMNS). This does NOT decide which candidates are
    trustworthy enough to actually cut the road; see lane_line_tracker.py.
    """
    h, w = marking_evidence.shape
    peaks = _confident_peaks(marking_evidence, drivable_mask, confident_threshold, min_peak_separation)
    claimed = set()
    candidates = []

    for c in range(w):
        for seed_row in peaks[c]:
            if (c, seed_row) in claimed:
                continue
            claimed.add((c, seed_row))

            left_cols, left_rows, left_conf = _walk_evidence(
                peaks, claimed, c, seed_row, -1, max_row_jump, search_radius, w)
            right_cols, right_rows, right_conf = _walk_evidence(
                peaks, claimed, c, seed_row, +1, max_row_jump, search_radius, w)

            row_by_col = np.empty(w, dtype=np.float32)
            confirmed_by_col = np.zeros(w, dtype=bool)
            row_by_col[c] = seed_row
            confirmed_by_col[c] = True
            for col, row, conf in zip(left_cols, left_rows, left_conf):
                row_by_col[col] = row
                confirmed_by_col[col] = conf
            for col, row, conf in zip(right_cols, right_rows, right_conf):
                row_by_col[col] = row
                confirmed_by_col[col] = conf

            confirmed_columns = int(confirmed_by_col.sum())
            if confirmed_columns >= min_confirmed_columns:
                candidates.append(CandidateLine(
                    seed_row=seed_row, seed_col=c,
                    row_by_col=row_by_col, confirmed_by_col=confirmed_by_col,
                    confirmed_columns=confirmed_columns))

    return candidates


def candidates_to_mask(candidates, grid_shape):
    """Rasterizes a list of CandidateLine (e.g. lane_line_tracker.LineTracker.
    get_trusted_lines()) into a boolean barrier mask, one True cell per
    column per line (row_by_col rounded to the nearest cell) -- the shape
    lane_segmentation.segment_lanes's barrier_mask parameter expects."""
    h, w = grid_shape
    mask = np.zeros((h, w), dtype=bool)
    for cand in candidates:
        rows = np.clip(np.round(cand.row_by_col).astype(np.int64), 0, h - 1)
        mask[rows, np.arange(w)] = True
    return mask
