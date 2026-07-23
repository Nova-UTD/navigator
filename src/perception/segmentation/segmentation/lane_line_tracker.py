#!/usr/bin/env python3
"""
lane_line_tracker.py — cross-tick confirmation and duplicate suppression for
candidate marking lines.

lane_barrier_fitting.extract_barrier_lines re-derives candidate lines from
scratch every publish tick with no memory of prior ticks. Live testing
found that trusting a candidate purely from one tick's evidence is too
easy for noise to clear -- a spurious run occasionally produces enough
confirmed columns to look real for a single frame, and since a trusted
candidate gets projected through the *entire* corridor, one bad tick can
slice the whole road into narrow phantom lanes. Confirmed live via an
RViz screenshot: the real divider is detected correctly, but other
independently-noisy candidates also clear the per-tick bar and get the
same trust -- a thin duplicate right next to the real line, and/or an
extra spurious cut elsewhere -- and which candidates clear the bar changes
tick to tick, so the published lane count flickered between correct and
garbage seconds apart.

This module tracks candidates across ticks, following this repo's existing
idiom for exactly this kind of problem
(multi_object_tracker_3d/trajectory.py's consecutive_missed_num: a
per-tracked-entity hit/miss counter compared against a configured
threshold). A candidate only becomes "trusted" (i.e. allowed to actually
cut the road) after being independently re-detected for several
consecutive ticks, and stays trusted through a few missed ticks rather
than flickering off immediately -- harder to lose trust than to gain it,
biasing toward stability once established. Independently, two candidates
-- new or already-trusted -- that end up closer together than a plausible
real lane width are recognized as the same physical line rather than kept
as separate cuts; the check runs on the whole tracked population every
tick, not just already-trusted lines, so a duplicate is compared to its
neighbor from the moment it appears rather than being given a chance to
climb to trust in isolation first.

Pure numpy, no rclpy/cv2 -- unit-testable standalone, same style as the
rest of this package.
"""

from dataclasses import dataclass

import numpy as np

from segmentation.bev_geometry import VEHICLE_COL, RESOLUTION

# How close a new tick's reference row must be to an existing track's for
# them to be considered the same line (not a new one). Deliberately much
# smaller than MIN_LANE_WIDTH_CELLS below, so ordinary tick-to-tick jitter
# in identifying a line can never itself conflate two genuinely distinct
# adjacent lines -- that job belongs solely to the dedup pass.
MATCH_TOLERANCE_CELLS = 3

# lane_grid_node's publish timer runs at 20Hz (create_timer(0.05, ...)).
# 6 consecutive matched ticks = 0.30s before a brand-new line starts
# cutting the road -- long enough that a single-tick noise spike (which
# would need to independently clear extract_barrier_lines's own admission
# filter AND land within match tolerance of itself 6 times running) is
# very unlikely, short enough that a real line doesn't take a perceptible
# amount of time to show up.
CONFIRM_HITS = 6

# 10 consecutive missed ticks = 0.50s of grace before a track (trusted or
# not) is dropped. Deliberately larger than CONFIRM_HITS -- harder to lose
# trust than to gain it, so a brief occlusion or a couple of weak frames
# doesn't instantly flicker a real, established line off.
MAX_CONSECUTIVE_MISSES = 10

# A real lane is ~3.0-3.5m wide (matches this repo's lane_width_m=3.5
# convention in lane_controlled_costmap_node.py/target_lane_selector.py/
# lane_change_node.py). The observed phantom duplicates sat 0.6-0.8m
# apart. 2.0m sits comfortably above the noise regime and comfortably
# below the real-lane regime, so it doesn't risk falsely merging a
# genuinely narrower real lane while still reliably catching a
# near-duplicate.
MIN_LANE_WIDTH_M = 2.0
MIN_LANE_WIDTH_CELLS = round(MIN_LANE_WIDTH_M / RESOLUTION)

# Near/ego/far sample columns for the dedup closeness check -- checking
# several spread-out points (not just the ego's own column) means two
# lines that only cross or converge briefly (e.g. near a merge) aren't
# falsely collapsed if they're genuinely separate along most of the
# corridor; a pair is only merged if it's close at every sampled point.
CHECK_COLUMNS = (30, 100, 270)

# Half-width (in columns) of the window averaged around VEHICLE_COL to get
# a track's reference row -- a window rather than a single column so
# per-column slope-estimation jitter can't cause spurious re-identification
# between ticks.
REF_WINDOW_CELLS = 5


@dataclass
class TrackedLine:
    """A candidate line's state across ticks. row_by_col/confirmed_by_col
    mirror lane_barrier_fitting.CandidateLine's fields (this class is
    duck-type compatible with candidates_to_mask, which only reads
    .row_by_col)."""
    id: int
    row_by_col: np.ndarray
    confirmed_by_col: np.ndarray
    reference_row: float
    hit_streak: int = 1
    miss_streak: int = 0
    trusted: bool = False


def _reference_row(row_by_col, vehicle_col=VEHICLE_COL, window=REF_WINDOW_CELLS):
    """Mean row over vehicle_col +/- window -- see module-level
    REF_WINDOW_CELLS docstring for why a window, not a single column."""
    lo = max(0, vehicle_col - window)
    hi = min(len(row_by_col), vehicle_col + window + 1)
    return float(np.mean(row_by_col[lo:hi]))


def _too_close(a, b, check_columns, min_lane_width_cells):
    """True if a and b are closer than min_lane_width_cells at every
    checked column where both have a defined (non-NaN) row -- requires at
    least 2 comparison columns to judge, so a single coincidental crossing
    point can't by itself trigger a merge."""
    diffs = []
    for c in check_columns:
        ra, rb = float(a.row_by_col[c]), float(b.row_by_col[c])
        if np.isnan(ra) or np.isnan(rb):
            continue
        diffs.append(abs(ra - rb))
    if len(diffs) < 2:
        return False
    return all(d < min_lane_width_cells for d in diffs)


class LineTracker:
    """Tracks candidate marking lines across publish ticks. Call update()
    once per tick with the current tick's candidates (e.g. from
    lane_barrier_fitting.extract_barrier_lines), then get_trusted_lines()
    to fetch the lines that have earned enough consecutive confirmations
    to actually cut the road."""

    def __init__(self, match_tolerance_cells=MATCH_TOLERANCE_CELLS,
                 confirm_hits=CONFIRM_HITS,
                 max_consecutive_misses=MAX_CONSECUTIVE_MISSES,
                 min_lane_width_cells=MIN_LANE_WIDTH_CELLS,
                 check_columns=CHECK_COLUMNS):
        self._match_tolerance_cells = match_tolerance_cells
        self._confirm_hits = confirm_hits
        self._max_consecutive_misses = max_consecutive_misses
        self._min_lane_width_cells = min_lane_width_cells
        self._check_columns = check_columns
        self._tracks = {}
        self._next_id = 0

    def update(self, candidates):
        """candidates: list of lane_barrier_fitting.CandidateLine for the
        current tick. Matches them against existing tracks, advances
        hit/miss streaks, promotes/drops tracks, seeds new ones for
        unmatched candidates, and runs duplicate suppression over the
        whole surviving population."""
        cand_refs = [_reference_row(c.row_by_col) for c in candidates]

        pairs = []
        for ci, cref in enumerate(cand_refs):
            for tid, t in self._tracks.items():
                d = abs(cref - t.reference_row)
                if d <= self._match_tolerance_cells:
                    pairs.append((d, ci, tid))
        pairs.sort(key=lambda p: p[0])

        matched_cand, matched_track = set(), set()
        for d, ci, tid in pairs:
            if ci in matched_cand or tid in matched_track:
                continue
            matched_cand.add(ci)
            matched_track.add(tid)
            t = self._tracks[tid]
            t.row_by_col = candidates[ci].row_by_col
            t.confirmed_by_col = candidates[ci].confirmed_by_col
            t.reference_row = cand_refs[ci]
            t.hit_streak += 1
            t.miss_streak = 0
            if t.hit_streak >= self._confirm_hits:
                t.trusted = True

        for tid, t in self._tracks.items():
            if tid not in matched_track:
                t.miss_streak += 1
                t.hit_streak = 0

        self._tracks = {tid: t for tid, t in self._tracks.items()
                         if t.miss_streak < self._max_consecutive_misses}

        for ci, c in enumerate(candidates):
            if ci not in matched_cand:
                self._tracks[self._next_id] = TrackedLine(
                    id=self._next_id, row_by_col=c.row_by_col,
                    confirmed_by_col=c.confirmed_by_col, reference_row=cand_refs[ci])
                self._next_id += 1

        self._dedup()

    def _dedup(self):
        """Runs over the full surviving population every call -- not just
        trusted lines -- so a duplicate is compared to its neighbor from
        the tick it appears, rather than being able to climb to trust in
        isolation first (see module docstring)."""
        order = sorted(
            self._tracks,
            key=lambda tid: (
                not self._tracks[tid].trusted,   # trusted lines win ties (False sorts first)
                -self._tracks[tid].hit_streak,    # then longer-established streak
                tid,                              # then older id
            ))

        removed = set()
        for i, id_a in enumerate(order):
            if id_a in removed:
                continue
            for id_b in order[i + 1:]:
                if id_b in removed:
                    continue
                if _too_close(self._tracks[id_a], self._tracks[id_b],
                               self._check_columns, self._min_lane_width_cells):
                    loser = self._tracks[id_b]
                    if loser.trusted:
                        # Demote rather than drop: avoids an instant
                        # one-tick disappearance from the published grid,
                        # and lets it re-earn trust independently if the
                        # lines diverge again (e.g. a merge that un-merges).
                        loser.trusted = False
                        loser.hit_streak = 0
                    else:
                        # Wasn't contributing to the output yet -- safe to
                        # drop outright before it can ever climb to trust.
                        removed.add(id_b)

        for tid in removed:
            del self._tracks[tid]

    def get_trusted_lines(self):
        """Returns the tracks currently trusted enough to cut the road."""
        return [t for t in self._tracks.values() if t.trusted]
