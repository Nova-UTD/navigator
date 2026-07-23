"""Unit tests for camera_lane_evidence.py — pure numpy/opencv, no ROS required.

Run with: python3 -m pytest src/perception/segmentation/test/test_camera_lane_evidence.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import camera_lane_evidence as cle


class _FakeLUT:
    """Minimal stand-in for bev_geometry.CamLUT: only the attributes
    camera_marking_evidence actually reads (gr, gc, valid), so the
    aggregation logic can be tested independent of real camera projection
    geometry (that's bev_geometry's own concern)."""

    def __init__(self, gr, gc, valid):
        self.gr = gr
        self.gc = gc
        self.valid = valid


def test_road_mask_from_semantic_matches_road_color_only():
    img = np.zeros((4, 4, 3), dtype=np.uint8)
    img[:, :2] = cle.ROAD_COLOR       # left half road
    img[:, 2:] = (70, 70, 70)         # right half building

    mask = cle.road_mask_from_semantic(img)

    assert mask[:, :2].all()
    assert not mask[:, 2:].any()


def test_marking_search_mask_includes_any_non_sky_class():
    """Deliberately classification-agnostic (see module docstring): earlier
    versions gated the top-hat detector behind an allowlist (road |
    sidewalk | pole | traffic sign) that grew every time live testing found
    PSPNet mislabeling real lane paint as yet another class -- most
    recently "train" (0, 80, 100), a class never previously seen in this
    context. Rather than add a fifth entry, the mask now searches
    everything except sky, so a class nobody anticipated is still
    searched."""
    img = np.zeros((4, 6, 3), dtype=np.uint8)
    img[:, 0:2] = cle.ROAD_COLOR
    img[:, 2:3] = (0, 80, 100)     # train -- previously unhandled, must be included
    img[:, 3:4] = (70, 70, 70)     # building -- must be included
    img[:, 4:5] = (0, 0, 142)      # car -- must be included
    img[:, 5:6] = cle.SKY_COLOR    # sky -- must NOT be included

    mask = cle.marking_search_mask_from_semantic(img)

    assert mask[:, 0:2].all()
    assert mask[:, 2:3].all()
    assert mask[:, 3:4].all()
    assert mask[:, 4:5].all()
    assert not mask[:, 5:6].any()


def test_marking_candidate_mask_finds_bright_line_on_dark_road():
    h, w = 120, 200
    gray_road = 40
    image = np.full((h, w, 3), gray_road, dtype=np.uint8)
    line_row = 60
    image[line_row - 1:line_row + 2, :] = 220  # bright thin horizontal line
    road_mask = np.ones((h, w), dtype=bool)

    candidate = cle.marking_candidate_mask(image, road_mask)

    # The line itself is flagged...
    assert candidate[line_row, 100]
    # ...but plain road well away from the line is not.
    assert not candidate[10, 100]
    assert not candidate[110, 100]


def test_marking_candidate_mask_ignores_smooth_shadow_gradient():
    """Regression test for the original bug's failure mode: a smoothly
    varying brightness field (e.g. a shadow gradient across the frame) has
    no structural bright-thin-line feature, and must not be flagged just
    because some pixels are locally the 'brightest' in the frame -- that
    relative-not-structural reasoning is exactly what made the LiDAR
    percentile approach fail."""
    h, w = 120, 200
    # Smooth linear gradient from dark to lighter, no line anywhere.
    gradient = np.linspace(30, 90, w, dtype=np.uint8)
    image = np.tile(gradient, (h, 1))
    image = np.stack([image, image, image], axis=-1)
    road_mask = np.ones((h, w), dtype=bool)

    candidate = cle.marking_candidate_mask(image, road_mask)

    assert not candidate.any()


def test_marking_candidate_mask_restricted_to_road_region():
    h, w = 120, 200
    image = np.full((h, w, 3), 40, dtype=np.uint8)
    image[58:62, :] = 220  # bright line spans the whole width
    road_mask = np.zeros((h, w), dtype=bool)
    road_mask[:, :100] = True  # only the left half is road

    candidate = cle.marking_candidate_mask(image, road_mask)

    assert candidate[60, 50]         # on the line, in the road region
    assert not candidate[60, 150]    # on the line, but off-road -> excluded


def test_camera_marking_evidence_ratio_and_min_hits_gate():
    grid_size = 10
    # Cell (2, 2): 4 projected pixels, 3 candidates -> trusted, ratio 0.75.
    # Cell (5, 5): 2 projected pixels, both candidates -> below min_hits (3), untrusted -> 0.
    gr = np.array([2, 2, 2, 2, 5, 5])
    gc = np.array([2, 2, 2, 2, 5, 5])
    valid = np.ones(6, dtype=bool)
    candidate_mask = np.zeros((1, 6), dtype=bool)
    candidate_mask[0, :3] = True   # 3 of cell (2,2)'s 4 pixels
    candidate_mask[0, 4:] = True   # both of cell (5,5)'s 2 pixels
    lut = _FakeLUT(gr, gc, valid)

    evidence, observed = cle.camera_marking_evidence(candidate_mask, lut, grid_size, min_hits=3)

    assert observed[2, 2]
    assert evidence[2, 2] == pytest.approx(0.75)
    assert not observed[5, 5]
    assert evidence[5, 5] == 0.0
    assert not observed[0, 0]  # never projected into at all


def test_camera_marking_evidence_no_valid_pixels():
    grid_size = 10
    lut = _FakeLUT(np.zeros(4, dtype=int), np.zeros(4, dtype=int),
                    np.zeros(4, dtype=bool))
    candidate_mask = np.zeros((1, 4), dtype=bool)

    evidence, observed = cle.camera_marking_evidence(candidate_mask, lut, grid_size)

    assert evidence.shape == (grid_size, grid_size)
    assert not evidence.any()
    assert not observed.any()


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
