"""Unit tests for yolopv2_preprocessing.py — no ROS, no torch, no model
required.

Run with: python3 -m pytest src/perception/segmentation/test/test_yolopv2_preprocessing.py
"""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import yolopv2_preprocessing as yln


def test_letterbox_exact_aspect_match_has_zero_padding():
    """Confirmed empirically live: an 800x600 (4:3) CARLA frame letterboxed
    to a 640-based target lands on an exact 640x480 fit with no padding
    needed at all -- this is the common case for this project's cameras,
    and worth pinning down as a regression test."""
    img = np.zeros((600, 800, 3), dtype=np.uint8)

    padded, ratio, pad = yln.letterbox(img, new_shape=640)

    assert padded.shape[:2] == (480, 640)
    assert ratio == pytest.approx(0.8)
    assert pad == (0.0, 0.0)


def test_letterbox_pads_a_mismatched_aspect_ratio():
    """A square source scaled against a square target always fits exactly
    (both dimensions scale identically) -- this needs a source whose
    scaled dimension actually lands off a stride-32 multiple to exercise
    padding at all. 600x500 scaled to fit a 640 target is height-
    constrained (scale factor 640/600), leaving the scaled width (533) 11
    short of the next stride-32 multiple (544)."""
    img = np.zeros((600, 500, 3), dtype=np.uint8)  # h=600, w=500

    padded, ratio, pad = yln.letterbox(img, new_shape=640, stride=32)

    dw, dh = pad
    assert dh == 0  # height already lands exactly on the target
    assert dw > 0  # width needs padding to reach the next stride-32 multiple
    assert padded.shape[:2] == (640, 544)  # height untouched, width padded up to a multiple of 32


def test_unletterbox_mask_recovers_original_shape_with_zero_padding():
    original_shape = (600, 800)
    mask = np.zeros((480, 640), dtype=bool)
    mask[100:110, 200:210] = True  # a small marked region

    recovered = yln.unletterbox_mask(mask, pad=(0.0, 0.0), original_shape=original_shape)

    assert recovered.shape == original_shape
    # The marked region should still be present, scaled up proportionally
    # (480->600 is 1.25x, 640->800 is 1.25x).
    assert recovered[int(100 * 1.25):int(110 * 1.25), int(200 * 1.25):int(210 * 1.25)].any()
    assert not recovered[0:50, 0:50].any()


def test_unletterbox_mask_crops_padding_before_resizing():
    """A mask with real content only in the unpadded center region should
    have that content correctly recovered at the original image's full
    extent once the padding is cropped off -- not squashed/offset by
    treating the padding as real content."""
    original_shape = (200, 200)
    mask = np.zeros((100, 120), dtype=bool)
    # Real content occupies the middle columns; columns 0:10 and 110:120
    # simulate left/right letterbox padding (never marked).
    mask[:, 10:110] = True

    recovered = yln.unletterbox_mask(mask, pad=(10.0, 0.0), original_shape=original_shape)

    assert recovered.shape == original_shape
    assert recovered.all()  # padding cropped away, remaining content fills the full recovered mask


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
