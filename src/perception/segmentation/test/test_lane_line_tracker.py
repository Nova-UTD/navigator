"""Unit tests for lane_line_tracker.py — pure numpy, no ROS required.

Run with: python3 -m pytest src/perception/segmentation/test/test_lane_line_tracker.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import lane_line_tracker as llt
from segmentation.lane_barrier_fitting import CandidateLine
from segmentation.bev_geometry import GRID_SIZE


def flat_candidate(row, grid_size=GRID_SIZE):
    """A candidate whose row estimate is the same at every column (a
    straight, flat line) -- enough to exercise the tracker's matching/
    streak/dedup logic without needing real evidence-grid machinery."""
    row_by_col = np.full(grid_size, float(row), dtype=np.float32)
    confirmed_by_col = np.ones(grid_size, dtype=bool)
    return CandidateLine(seed_row=row, seed_col=0, row_by_col=row_by_col,
                          confirmed_by_col=confirmed_by_col, confirmed_columns=grid_size)


def test_new_candidate_not_trusted_until_confirm_hits():
    tracker = llt.LineTracker()
    for i in range(llt.CONFIRM_HITS - 1):
        tracker.update([flat_candidate(150)])
        assert tracker.get_trusted_lines() == [], f"trusted too early at tick {i + 1}"

    tracker.update([flat_candidate(150)])
    assert len(tracker.get_trusted_lines()) == 1


def test_a_miss_before_confirmation_resets_the_streak():
    tracker = llt.LineTracker()
    for _ in range(llt.CONFIRM_HITS - 1):
        tracker.update([flat_candidate(150)])
    # One miss right before what would have been the confirming tick.
    tracker.update([])
    track = next(iter(tracker._tracks.values()))
    assert track.hit_streak == 0
    assert not track.trusted

    # Must climb the full CONFIRM_HITS again from here, not resume where it left off.
    for i in range(llt.CONFIRM_HITS - 1):
        tracker.update([flat_candidate(150)])
        assert tracker.get_trusted_lines() == []
    tracker.update([flat_candidate(150)])
    assert len(tracker.get_trusted_lines()) == 1


def test_trusted_line_survives_misses_up_to_the_limit_then_drops():
    tracker = llt.LineTracker()
    for _ in range(llt.CONFIRM_HITS):
        tracker.update([flat_candidate(150)])
    assert len(tracker.get_trusted_lines()) == 1

    for _ in range(llt.MAX_CONSECUTIVE_MISSES - 1):
        tracker.update([])
        assert len(tracker.get_trusted_lines()) == 1, "should still be trusted before the limit"

    tracker.update([])  # this miss reaches MAX_CONSECUTIVE_MISSES
    assert tracker.get_trusted_lines() == []
    assert tracker._tracks == {}


def test_two_close_candidates_collapse_to_one_trusted_line():
    tracker = llt.LineTracker()
    close_offset = llt.MIN_LANE_WIDTH_CELLS - 2  # closer than a plausible real lane
    for _ in range(llt.CONFIRM_HITS + 2):
        tracker.update([flat_candidate(150), flat_candidate(150 + close_offset)])

    trusted = tracker.get_trusted_lines()
    assert len(trusted) == 1


def test_two_well_separated_candidates_both_get_trusted():
    tracker = llt.LineTracker()
    wide_offset = llt.MIN_LANE_WIDTH_CELLS + 5  # further apart than a real lane needs to be
    for _ in range(llt.CONFIRM_HITS + 2):
        tracker.update([flat_candidate(150), flat_candidate(150 + wide_offset)])

    trusted = tracker.get_trusted_lines()
    assert len(trusted) == 2


def test_established_line_beats_a_newcomer_that_lands_too_close():
    tracker = llt.LineTracker()
    for _ in range(llt.CONFIRM_HITS + 3):
        tracker.update([flat_candidate(150)])
    assert len(tracker.get_trusted_lines()) == 1

    close_offset = llt.MIN_LANE_WIDTH_CELLS - 2
    for _ in range(llt.CONFIRM_HITS + 2):
        tracker.update([flat_candidate(150), flat_candidate(150 + close_offset)])

    trusted = tracker.get_trusted_lines()
    assert len(trusted) == 1
    assert trusted[0].reference_row == pytest.approx(150, abs=1)


def test_trusted_pair_that_converges_is_demoted_not_removed():
    tracker = llt.LineTracker()
    wide_offset = llt.MIN_LANE_WIDTH_CELLS + 5
    for _ in range(llt.CONFIRM_HITS + 2):
        tracker.update([flat_candidate(150), flat_candidate(150 + wide_offset)])
    assert len(tracker.get_trusted_lines()) == 2
    track_ids_before = set(tracker._tracks.keys())

    # Converge gradually (each step within MATCH_TOLERANCE_CELLS) so the
    # second line stays matched/updated -- and thus still trusted -- as it
    # approaches, the same way a real merging lane would move tick to
    # tick, rather than teleporting close in one jump (which would just
    # orphan the old track instead of demonstrating a converging one).
    # Stop at the *first* tick that crosses into "too close" -- one more
    # step beyond this would find the now-untrusted loser still
    # overlapping and correctly drop it outright (a separate, later
    # behavior, not what this test is checking).
    step = llt.MATCH_TOLERANCE_CELLS
    offset = wide_offset - step
    tracker.update([flat_candidate(150), flat_candidate(150 + offset)])
    assert offset >= llt.MIN_LANE_WIDTH_CELLS, "first step should not yet be close enough to dedup"

    offset -= step
    assert offset < llt.MIN_LANE_WIDTH_CELLS, "second step should be the one that crosses into too-close"
    tracker.update([flat_candidate(150), flat_candidate(150 + offset)])

    assert len(tracker.get_trusted_lines()) == 1
    # The demoted line is still tracked (not dropped outright), just unconfirmed.
    assert set(tracker._tracks.keys()) == track_ids_before


def test_small_jitter_keeps_identity_bigger_jump_starts_a_new_track():
    tracker = llt.LineTracker()
    jitter = llt.MATCH_TOLERANCE_CELLS  # at the edge of tolerance, still a match
    # Alternates rather than drifting: each step's distance from the
    # *previous* accepted value (what matching actually compares against)
    # stays at exactly the tolerance, never compounding across ticks.
    rows = [150, 150 + jitter, 150, 150 + jitter, 150, 150]
    for row in rows:
        tracker.update([flat_candidate(row)])

    assert len(tracker._tracks) == 1
    assert len(tracker.get_trusted_lines()) == 1  # identity preserved across jitter -> streak reached CONFIRM_HITS

    # Must clear MIN_LANE_WIDTH_CELLS too, not just MATCH_TOLERANCE_CELLS --
    # otherwise the new track would be created and then immediately
    # deduped away for sitting too close to the established one within
    # this same update() call, making it indistinguishable from "no new
    # track" by the final count alone.
    far_jump = llt.MIN_LANE_WIDTH_CELLS + 5
    tracker.update([flat_candidate(150 + far_jump)])

    assert len(tracker._tracks) == 2  # original track missed this tick, a new one seeded


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
