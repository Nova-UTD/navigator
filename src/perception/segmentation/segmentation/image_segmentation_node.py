"""
image_segmentation_node.py
Runs PSPNet (mmseg v1.x) on 4 CARLA cameras in a round-robin background thread.
Callbacks only store the latest frame — no blocking inference in the spin thread.
"""

import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from mmseg.apis import inference_model, init_model

# input_topic → output_topic (bridge remaps /carla/hero/rgb_*/image → /cameras/camera*)
CAMERAS = [
    ('/cameras/camera0', '/semantic/front'),
    ('/cameras/camera1', '/semantic/right'),
    ('/cameras/camera2', '/semantic/back'),
    ('/cameras/camera3', '/semantic/left'),
]

_CONFIG = '/usr/local/lib/python3.10/dist-packages/mmseg/.mim/configs/pspnet/pspnet_r18-d8_4xb2-80k_cityscapes-512x1024.py'
_CKPT   = '/navigator_binaries/pspnet_r18-d8_512x1024_80k_cityscapes_20201225_021458-09ffa746.pth'

# Cityscapes 19-class → RGB (for downstream classification)
_PALETTE = {
    0:  (128,  64, 128),  # road
    1:  (244,  35, 232),  # sidewalk
    2:  ( 70,  70,  70),  # building
    3:  (102, 102, 156),  # wall
    4:  (190, 153, 153),  # fence
    5:  (153, 153, 153),  # pole
    6:  (250, 170,  30),  # traffic light
    7:  (220, 220,   0),  # traffic sign
    8:  (107, 142,  35),  # vegetation
    9:  (145, 170, 100),  # terrain
    10: ( 70, 130, 180),  # sky
    11: (220,  20,  60),  # person
    12: (255,   0,   0),  # rider
    13: (  0,   0, 142),  # car
    14: (  0,   0,  70),  # truck
    15: (  0,  60, 100),  # bus
    16: (  0,  80, 100),  # train
    17: (  0,   0, 230),  # motorcycle
    18: (119,  11,  32),  # bicycle
}

def _class_ids_to_rgb(class_ids: np.ndarray) -> np.ndarray:
    H, W = class_ids.shape
    rgb = np.zeros((H, W, 3), dtype=np.uint8)
    for cid, color in _PALETTE.items():
        mask = class_ids == cid
        rgb[mask] = color
    return rgb


class ImageSegmentationNode(Node):

    def __init__(self):
        super().__init__('image_segmentation_node')
        self.get_logger().info('Loading PSPNet on CPU…')
        self.model  = init_model(_CONFIG, _CKPT, device='cpu')
        self.bridge = CvBridge()
        self.get_logger().info('PSPNet ready.')

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        # Per-camera: latest raw frame + publisher
        self._latest = {}   # topic → (msg | None, publisher)
        self._lock   = threading.Lock()
        self._order  = [t for t, _ in CAMERAS]

        for in_topic, out_topic in CAMERAS:
            pub = self.create_publisher(Image, out_topic, 1)
            self._latest[in_topic] = (None, pub)
            self.create_subscription(Image, in_topic,
                lambda msg, t=in_topic: self._store(msg, t), image_qos)
            self.get_logger().info(f'  {in_topic} → {out_topic}')

        self._idx = 0
        threading.Thread(target=self._loop, daemon=True).start()

    # ── callbacks: just store, never block ──────────────────────────────
    def _store(self, msg: Image, topic: str):
        with self._lock:
            _, pub = self._latest[topic]
            self._latest[topic] = (msg, pub)

    # ── inference loop: round-robin all 4 cameras ───────────────────────
    def _loop(self):
        import time
        n = len(self._order)
        self.get_logger().info('Inference thread running (round-robin 4 cameras).')
        while True:
            cam = self._order[self._idx]
            with self._lock:
                msg, pub = self._latest[cam]

            if msg is not None:
                try:
                    img       = self.bridge.imgmsg_to_cv2(msg, 'rgb8')[:, :, :3]
                    result    = inference_model(self.model, img)
                    class_ids = result.pred_sem_seg.data[0].cpu().numpy()
                    rgb_out   = _class_ids_to_rgb(class_ids)
                    out_msg   = self.bridge.cv2_to_imgmsg(rgb_out, encoding='rgb8')
                    out_msg.header = msg.header
                    pub.publish(out_msg)
                except Exception as e:
                    self.get_logger().error(f'Inference error on {cam}: {e}')
            else:
                time.sleep(0.05)

            self._idx = (self._idx + 1) % n


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ImageSegmentationNode())


if __name__ == '__main__':
    main()
