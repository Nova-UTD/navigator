#!/usr/bin/env python3
"""
yolopv2_lane_node.py — real-time lane-line segmentation via a pretrained
YOLOPv2 model, replacing the front camera's hand-built top-hat marking
detector.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

Model: YOLOPv2 (https://github.com/CAIC-AD/YOLOPv2), official pretrained
release weights (yolopv2.pt), trained on BDD100K real-world driving data.
Weights file location on this project's hosts: /navigator_binaries/yolopv2.pt
(same convention as lane_type_detector's lane_detector.pt -- a host-mounted
binaries directory, not tracked in this git repo). Loaded once at node
startup via torch.jit.load (self-contained TorchScript, no separate model
definition code required).

How it works: one shared convolutional backbone feeds three task heads --
object detection, drivable-area segmentation, and lane-line segmentation.
This node only uses the drivable-area and lane-line heads. Per frame: the
raw camera image is letterboxed (resized + padded) to the model's input
size, run through the model in a single forward pass, and the two
resulting per-pixel score maps are thresholded into binary masks and
mapped back to the raw image's native resolution (undoing the letterbox --
see yolopv2_preprocessing.py) before publishing.

Today's from-scratch approach (morphological top-hat filter + geometric
barrier-fitting + temporal confirmation, all in camera_lane_evidence.py/
lane_barrier_fitting.py/lane_line_tracker.py) went through several
redesigns live and never reached a stable result. Offline validation
against a real captured CARLA front-camera frame found YOLOPv2 (a
real-time multi-task model pretrained on BDD100K -- drivable area +
lane-line segmentation + object detection) produces a dramatically better,
cleaner lane-line mask than anything built by hand today, and than the
project's other existing lane_detector.pt model (which found zero
detections on the same frame). Steady-state inference is ~7ms/frame
(139 FPS) on cuda:1 once warmed up -- not a performance concern.

Model is loaded via torch.jit.load from a self-contained TorchScript file
(/navigator_binaries/yolopv2.pt, the official release weights) -- no need
to vendor YOLOPv2's own training/repo code, only the tiny mask-extraction
pixel math its own utils.py does (see yolopv2_preprocessing.py, which
reimplements it using this project's own actual letterbox padding rather
than YOLOPv2's hardcoded crop constants -- those assume a different input
aspect ratio than this project's 4:3 camera frames).

Front camera only for this phase: YOLOPv2 is pretrained on forward-facing
dashcam data; our side cameras are mounted at a ~70 degree yaw (a real
domain shift from what it was trained on), and validating it there needs
its own live test before trusting it. camera_lane_evidence.py's top-hat
approach stays in place for the right/left cameras in lane_grid_node.py.

Structure mirrors image_segmentation_node.py (the only other model-loading
node in this package) exactly: model loaded once in __init__ on cuda:1
(same GPU as PSPNet, kept off GPU0 to avoid contending with CARLA's own
rendering), subscription callback only stores the latest frame, a single
daemon background thread does stamp-gated inference so a cached frame is
never reprocessed/re-published in a tight loop.

Publishes two topics: /lane_mask/front (mono8 binary mask, what
lane_grid_node actually consumes) and /lane_mask/front/viz (bgr8, a
human-viewable overlay -- green drivable area + red lane lines drawn over
the raw camera frame -- for direct visual verification in RViz, matching
the same green/red visualization style used in this model's own offline
demo tooling).
"""

import threading
import time

import cv2
import numpy as np
import rclpy
import torch
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image

from segmentation.yolopv2_preprocessing import letterbox, unletterbox_mask

_WEIGHTS = '/navigator_binaries/yolopv2.pt'
_RAW_TOPIC = '/cameras/camera0'
_MASK_TOPIC = '/lane_mask/front'
_VIZ_TOPIC = '/lane_mask_viz/front'
_LANE_LINE_THRESHOLD = 0.5  # model's ll head output is already probability-like (0-1); round() at 0.5

_DRIVABLE_COLOR_BGR = (0, 200, 0)    # green
_LANE_LINE_COLOR_BGR = (0, 0, 230)   # red
_DRIVABLE_ALPHA = 0.5                # blend weight for the drivable-area fill


def _make_overlay(raw_bgr, drivable_mask, lane_mask):
    """raw_bgr: HxWx3 uint8. drivable_mask/lane_mask: HxW bool, already at
    raw_bgr's own resolution. Returns an HxWx3 uint8 overlay: a
    semi-transparent green fill over the drivable area, with lane-line
    pixels drawn solid red on top -- same visual language as YOLOPv2's own
    offline demo visualization, so a live RViz view reads the same way the
    offline validation frame did.
    """
    overlay = raw_bgr.copy()
    if drivable_mask.any():
        tint = np.full_like(raw_bgr, _DRIVABLE_COLOR_BGR)
        blended = cv2.addWeighted(raw_bgr, 1 - _DRIVABLE_ALPHA, tint, _DRIVABLE_ALPHA, 0)
        overlay[drivable_mask] = blended[drivable_mask]
    overlay[lane_mask] = _LANE_LINE_COLOR_BGR
    return overlay


class Yolopv2LaneNode(Node):

    def __init__(self):
        super().__init__('yolopv2_lane_node')
        self.get_logger().info("Loading YOLOPv2 on GPU (cuda:1, same as PSPNet, kept off GPU0)…")
        self.device = torch.device('cuda:1')
        self.model = torch.jit.load(_WEIGHTS)
        self.model = self.model.to(self.device).half()
        self.model.eval()
        self.bridge = CvBridge()
        self.get_logger().info('YOLOPv2 ready.')

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self._latest = None
        self._lock = threading.Lock()
        self.create_subscription(Image, _RAW_TOPIC, self._store, image_qos)

        self.lane_pub = self.create_publisher(Image, _MASK_TOPIC, 1)
        self.viz_pub = self.create_publisher(Image, _VIZ_TOPIC, 1)

        threading.Thread(target=self._loop, daemon=True).start()
        self.get_logger().info(f'  {_RAW_TOPIC} → {_MASK_TOPIC}, {_VIZ_TOPIC}')

    # ── callback: just store, never block ───────────────────────────────
    def _store(self, msg: Image):
        with self._lock:
            self._latest = msg

    # ── inference loop ───────────────────────────────────────────────────
    def _loop(self):
        # Same reasoning as image_segmentation_node.py's inference loop:
        # on GPU, inference is fast enough that without this check we'd
        # re-process and re-publish the same cached frame far faster than
        # new camera frames actually arrive.
        last_stamp = None
        self.get_logger().info('Inference thread running.')
        while True:
            with self._lock:
                msg = self._latest

            stamp = msg.header.stamp if msg is not None else None
            if msg is not None and stamp != last_stamp:
                last_stamp = stamp
                try:
                    self._infer_and_publish(msg)
                except Exception as e:
                    self.get_logger().error(f'Inference error: {e}')
            else:
                time.sleep(0.01)

    def _infer_and_publish(self, msg: Image):
        raw = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        original_shape = raw.shape[:2]

        padded, ratio, pad = letterbox(raw)
        img_in = padded[:, :, ::-1].transpose(2, 0, 1).copy()  # BGR -> RGB, HWC -> CHW
        img_t = torch.from_numpy(img_in).to(self.device).half() / 255.0
        img_t = img_t.unsqueeze(0)

        with torch.no_grad():
            [_pred, _anchor_grid], seg, ll = self.model(img_t)

        lane_line = (ll.squeeze(0).squeeze(0) >= _LANE_LINE_THRESHOLD).cpu().numpy()
        lane_mask = unletterbox_mask(lane_line, pad, original_shape)

        out_msg = self.bridge.cv2_to_imgmsg((lane_mask * 255).astype(np.uint8), encoding='mono8')
        out_msg.header = msg.header
        self.lane_pub.publish(out_msg)

        # Visualization branch: cheap on top of the forward pass we already
        # ran (just one more argmax over the seg head's 2 channels).
        drivable = torch.max(seg, 1)[1].squeeze(0).cpu().numpy().astype(bool)
        drivable_mask = unletterbox_mask(drivable, pad, original_shape).astype(bool)
        overlay = _make_overlay(raw, drivable_mask, lane_mask.astype(bool))
        viz_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        viz_msg.header = msg.header
        self.viz_pub.publish(viz_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Yolopv2LaneNode())


if __name__ == '__main__':
    main()
