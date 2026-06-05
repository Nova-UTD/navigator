#!/usr/bin/env python3
"""
hybrid_drivable_grid_node.py

Fuses 4-camera PSPNet semantic segmentation with the HD-map drivable surface
to produce a real-time /grid/drivable OccupancyGrid at ~10 Hz.

Replaces the MapManager static output (~0.7 Hz) with a faster, perception-aware
layer that can detect construction zones, dynamic obstacles, and other real-time
changes within road lanes.

Fusion rule:
  - HD map boundary cells (value 100) are NEVER overridden — legal road
    boundaries are law.
  - Within HD-map-drivable cells (value 0), camera cost is applied:
      road pixel -> 0  (confirms drivable)
      obstacle/sidewalk  -> raised cost up to 100
  - Cells not visible to any camera retain their HD map value.
  - Before HD map is received, a camera-only grid is published (safe fallback).

Inputs:
  /grid/drivable/hdmap               MapManager output, remapped in launch
  /semantic/{front,right,back,left}  PSPNet RGB seg from image_segmentation_node

Output:
  /grid/drivable   300x300 OccupancyGrid, 0.2m/cell, base_link frame
"""

import threading
import numpy as np
import cv2
import rclpy
from rclpy.node       import Node
from rclpy.time       import Time
from nav_msgs.msg     import OccupancyGrid
from sensor_msgs.msg  import Image, CameraInfo
from cv_bridge        import CvBridge
from scipy.spatial.transform import Rotation
from tf2_ros          import Buffer, TransformListener, TransformException

# -- Grid parameters (must match MapManager config) ---------------------------
GRID_SIZE  = 300
RESOLUTION = 0.2      # m per cell
ORIGIN_X   = -20.0    # forward offset of grid origin (base_link)
ORIGIN_Y   = -30.0    # lateral offset of grid origin (base_link)
MAX_DIST   = 40.0     # ignore ground hits beyond this range (m)

# -- Camera definitions -------------------------------------------------------
_CAMERAS = [
    ('front', 'hero/rgb_front', '/semantic/front', '/carla/hero/rgb_front/camera_info'),
    ('right', 'hero/rgb_right', '/semantic/right', '/carla/hero/rgb_right/camera_info'),
    ('back',  'hero/rgb_back',  '/semantic/back',  '/carla/hero/rgb_back/camera_info'),
    ('left',  'hero/rgb_left',  '/semantic/left',  '/carla/hero/rgb_left/camera_info'),
]

# -- Cityscapes RGB -> drivability cost LUT -----------------------------------
# -1 = skip (sky / no ground intersection)
#  0 = confirmed drivable (road surface)
# 15 = low-cost passable terrain (grass edge, gravel)
# 30 = soft avoid (traffic light / sign furniture)
# 50 = uncertain (pole base, ambiguous)
# 60 = prefer avoid (dense vegetation overhang)
# 80 = non-drivable boundary (sidewalk)
#100 = hard obstacle (building, vehicle, person, fence)
_DRIVE_OCC = {
    (128,  64, 128):   0,   # road
    (244,  35, 232):  80,   # sidewalk
    ( 70,  70,  70): 100,   # building
    (102, 102, 156): 100,   # wall
    (190, 153, 153): 100,   # fence
    (153, 153, 153):  50,   # pole
    (250, 170,  30):  30,   # traffic light
    (220, 220,   0):  30,   # traffic sign
    (107, 142,  35):  60,   # vegetation
    (145, 170, 100):  15,   # terrain
    ( 70, 130, 180):  -1,   # sky (no ground projection)
    (220,  20,  60): 100,   # person
    (255,   0,   0): 100,   # rider
    (  0,   0, 142): 100,   # car
    (  0,   0,  70): 100,   # truck
    (  0,  60, 100): 100,   # bus
    (  0,  80, 100): 100,   # train
    (  0,   0, 230): 100,   # motorcycle
    (119,  11,  32): 100,   # bicycle
}

def _build_drive_lut() -> np.ndarray:
    lut = np.full((256, 256, 256), -1, dtype=np.int8)
    for (r, g, b), v in _DRIVE_OCC.items():
        lut[r, g, b] = np.int8(v)
    return lut

_DRIVE_LUT = _build_drive_lut()   # built once at import time


class _CamLUT:
    __slots__ = ('gp_gc', 'gp_gr', 'gp_valid', 'img_h', 'img_w')


class HybridDrivableGridNode(Node):

    def __init__(self):
        super().__init__('hybrid_drivable_grid_node')
        self.bridge = CvBridge()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self._lock       = threading.Lock()

        self._K   : dict = {}   # camera name -> 3x3 K
        self._lut : dict = {}   # camera name -> _CamLUT
        self._sem : dict = {}   # camera name -> (H,W,3) uint8

        self._hdmap: np.ndarray | None = None   # latest HD map (300x300) int8

        qos_be  = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=10)

        for name, _, sem_t, info_t in _CAMERAS:
            self.create_subscription(CameraInfo, info_t,
                lambda m, n=name: self._cb_info(m, n), qos_rel)
            self.create_subscription(Image, sem_t,
                lambda m, n=name: self._cb_sem(m, n), qos_be)

        self.create_subscription(OccupancyGrid, '/grid/drivable/hdmap',
            self._cb_hdmap, 1)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/drivable', 10)

        self.create_timer(1.0, self._lut_timer)      # retry LUT until all built
        self.create_timer(0.1, self._publish_loop)   # publish at 10 Hz

        self.get_logger().info('HybridDrivableGridNode ready.')

    # -------------------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------------------

    def _cb_info(self, msg: CameraInfo, name: str):
        if name in self._K:
            return
        with self._lock:
            self._K[name] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info(f'[{name}] CameraInfo stored.')

    def _cb_sem(self, msg: Image, name: str):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self._lock:
            self._sem[name] = img

    def _cb_hdmap(self, msg: OccupancyGrid):
        arr = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        with self._lock:
            self._hdmap = arr

    # -------------------------------------------------------------------------
    # Ground-plane LUT
    # -------------------------------------------------------------------------

    def _lut_timer(self):
        for name, frame, _, _ in _CAMERAS:
            if name not in self._K:
                continue
            if name in self._lut and self._lut[name].gp_valid.any():
                continue
            self._try_build_lut(name, frame)

    def _try_build_lut(self, name: str, cam_frame: str):
        if not self.tf_buffer.can_transform('base_link', cam_frame, Time()):
            self.get_logger().warn(f'[{name}] TF not yet available.',
                                   throttle_duration_sec=5.0)
            return
        try:
            tf = self.tf_buffer.lookup_transform('base_link', cam_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f'[{name}] TF lookup failed: {ex}',
                                   throttle_duration_sec=5.0)
            return

        K  = self._K[name]
        q  = tf.transform.rotation
        tr = tf.transform.translation

        R_cb   = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix().astype(np.float64)
        t_base = np.array([tr.x, tr.y, tr.z], dtype=np.float64)

        sem = self._sem.get(name)
        H, W = sem.shape[:2] if sem is not None else (600, 800)

        K_inv = np.linalg.inv(K)
        us, vs = np.meshgrid(np.arange(W, dtype=np.float64),
                             np.arange(H, dtype=np.float64))
        uvh      = np.stack([us.ravel(), vs.ravel(), np.ones(H * W)])
        rays_cam = (K_inv @ uvh).T
        ray_base = (R_cb @ rays_cam.T).T.astype(np.float32)

        ray_z = ray_base[:, 2]
        safe  = np.abs(ray_z) > 1e-6
        lam   = np.where(safe, -t_base[2] / ray_z, 0.0)
        pts   = t_base + lam[:, np.newaxis] * ray_base

        gc     = ((pts[:, 0] - ORIGIN_X) / RESOLUTION).astype(np.int32)
        gr     = ((pts[:, 1] - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        dist2d = np.hypot(pts[:, 0], pts[:, 1])

        gp_valid = (
            safe & (lam > 0) & (dist2d < MAX_DIST) &
            (gc >= 0) & (gc < GRID_SIZE) &
            (gr >= 0) & (gr < GRID_SIZE)
        )

        lut          = _CamLUT()
        lut.gp_gc    = gc
        lut.gp_gr    = gr
        lut.gp_valid = gp_valid
        lut.img_h    = H
        lut.img_w    = W

        with self._lock:
            self._lut[name] = lut

        self.get_logger().info(
            f'[{name}] LUT built -- {gp_valid.sum()}/{H*W} ground pixels.')

    # -------------------------------------------------------------------------
    # Camera projection -> drivability layer
    # -------------------------------------------------------------------------

    def _project_cameras(self, sem_imgs: dict, luts: dict) -> np.ndarray:
        """Returns (GRID_SIZE, GRID_SIZE) int8; -1 = no camera data."""
        layer    = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
        coverage = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)

        for name, _, _, _ in _CAMERAS:
            lut = luts.get(name)
            sem = sem_imgs.get(name)
            if lut is None or sem is None:
                continue

            if sem.shape[0] != lut.img_h or sem.shape[1] != lut.img_w:
                sem = cv2.resize(sem, (lut.img_w, lut.img_h),
                                 interpolation=cv2.INTER_NEAREST)

            flat = sem.reshape(-1, 3)
            occ  = _DRIVE_LUT[flat[:, 0], flat[:, 1], flat[:, 2]]
            use  = lut.gp_valid & (occ != -1)
            if not use.any():
                continue

            rows = lut.gp_gr[use]
            cols = lut.gp_gc[use]
            vals = occ[use]

            blend   = coverage[rows, cols]
            nb_mask = ~blend

            if nb_mask.any():
                layer[rows[nb_mask], cols[nb_mask]] = vals[nb_mask]
            if blend.any():
                layer[rows[blend], cols[blend]] = (
                    (layer[rows[blend], cols[blend]].astype(np.int16) +
                     vals[blend].astype(np.int16)) // 2
                ).astype(np.int8)

            coverage[rows, cols] = True

        return layer

    # -------------------------------------------------------------------------
    # Publish loop
    # -------------------------------------------------------------------------

    def _publish_loop(self):
        with self._lock:
            sem_snap   = dict(self._sem)
            lut_snap   = dict(self._lut)
            hdmap_snap = self._hdmap.copy() if self._hdmap is not None else None

        cam_layer = self._project_cameras(sem_snap, lut_snap)

        if hdmap_snap is not None:
            # Fuse: HD map provides road boundary law; camera augments within
            # road cells only.  A cell the HD map marks as not-drivable (100)
            # is NEVER reduced by camera data.
            output = hdmap_snap.copy()

            drivable = (output == 0)   # cells inside legal lane
            known    = (cam_layer >= 0)
            augment  = drivable & known
            if augment.any():
                output[augment] = np.maximum(
                    output[augment], cam_layer[augment])
        else:
            # HD map not yet loaded -- publish camera-only layer so planner
            # is not completely blind during map manager startup.
            cam_known = (cam_layer >= 0)
            output    = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)
            output[cam_known] = cam_layer[cam_known]
            self.get_logger().warn(
                'HD map not yet received; publishing camera-only drivable grid.',
                throttle_duration_sec=5.0)

        msg                           = OccupancyGrid()
        msg.header.stamp              = self.get_clock().now().to_msg()
        msg.header.frame_id           = 'base_link'
        msg.info.resolution           = RESOLUTION
        msg.info.width                = GRID_SIZE
        msg.info.height               = GRID_SIZE
        msg.info.origin.position.x    = ORIGIN_X
        msg.info.origin.position.y    = ORIGIN_Y
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data                      = output.ravel().tolist()

        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Publish failed (transient): {e}',
                                   throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = HybridDrivableGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
