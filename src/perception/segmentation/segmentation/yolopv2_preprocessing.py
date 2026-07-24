#!/usr/bin/env python3
"""
yolopv2_preprocessing.py — letterbox resize and its inverse, for feeding a
camera frame into YOLOPv2 and mapping its output mask back to native pixel
coordinates.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Model: YOLOPv2 (https://github.com/CAIC-AD/YOLOPv2), official pretrained
release weights (yolopv2.pt) at /navigator_binaries/yolopv2.pt -- see
yolopv2_lane_node.py (the node that actually loads and runs the model) for
the full explanation of how the model works. This module only holds the
pure pixel-math pre/post-processing YOLOPv2's own preprocessing convention
requires: resize+pad the input to the model's expected size (letterbox),
and invert that resize+pad on the model's output mask so it lines back up
with the original, un-letterboxed camera frame.

Split out from yolopv2_lane_node.py so this pure cv2/numpy pixel math is
unit-testable without pulling in torch/rclpy/cv_bridge -- same separation
of concerns as camera_lane_evidence.py (pure) vs. lane_grid_node.py (ROS
node), elsewhere in this package.

Pure numpy/opencv, no rclpy/torch -- unit-testable standalone.
"""

import cv2
import numpy as np

DEFAULT_MODEL_INPUT_SIZE = 640
DEFAULT_STRIDE = 32


def letterbox(img, new_shape=DEFAULT_MODEL_INPUT_SIZE, color=(114, 114, 114), stride=DEFAULT_STRIDE):
    """Resize+pad img to fit within new_shape (a single int -> square
    target) while preserving aspect ratio, padding the shorter dimension
    to a stride-multiple -- same algorithm YOLOPv2's own preprocessing
    uses (simplified here to the one mode we need: auto/minimum-rectangle,
    scaleup allowed).

    Returns (padded_img, ratio, (pad_w, pad_h)). For this project's 800x600
    CARLA camera frames specifically, this produces an exact 640x480 fit
    with zero padding (4:3 matches 4:3) -- confirmed empirically -- but the
    general padding math is kept so this doesn't silently break if the
    camera resolution ever changes.
    """
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)
    shape = img.shape[:2]  # h, w

    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (int(round(shape[1] * r)), int(round(shape[0] * r)))  # w, h
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw, dh = dw % stride, dh % stride
    dw /= 2
    dh /= 2

    if shape[::-1] != new_unpad:
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    return img, r, (dw, dh)


def unletterbox_mask(mask, pad, original_shape):
    """Inverse of letterbox() for a mask that came out of the model at the
    same resolution as the letterboxed input (confirmed empirically for
    YOLOPv2: its seg/lane-line heads output at input resolution, no
    internal up/downsampling -- unlike YOLOPv2's own demo code, which
    assumes a fixed input aspect ratio and hardcodes its own crop
    constants accordingly; that assumption doesn't hold for this
    project's 4:3 camera frames, so this crops using the actual pad
    values from our own letterbox() call instead).

    Crops off the padding, then resizes the remaining (real-content)
    region up to original_shape -- so the result is pixel-aligned with the
    raw, pre-letterbox camera frame. mask: 2D array. pad: (pad_w, pad_h)
    exactly as returned by letterbox() for the same frame. original_shape:
    (h, w) of the raw camera image.
    """
    dw, dh = pad
    h, w = mask.shape
    top, bottom = int(round(dh - 0.1)), h - int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), w - int(round(dw + 0.1))
    cropped = mask[top:bottom, left:right]
    out_h, out_w = original_shape
    return cv2.resize(cropped.astype(np.uint8), (out_w, out_h), interpolation=cv2.INTER_NEAREST)
