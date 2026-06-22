#!/usr/bin/env python3
"""
hybrid_perception_grid_node.py

Fuses 4-camera semantic segmentation + segmented LiDAR into a 300x300 BEV
occupancy grid at /grid/occupancy/current.

Camera path  : /semantic/{front,right,back,left} (PSPNet RGB output)
               -> ground-plane ray-cast LUT -> grid cells
LiDAR path   : /lidar/filtered (PointCloud2 in base_link)
               -> project each point onto nearest camera image for semantic label
               -> improved 3-tier height fallback for dead zones (ground/curb/obstacle)
Output       : /grid/occupancy/current  300x300, 0.2 m/cell, frame=base_link

TF strategy  : 1-Hz timer retries every camera's LUT until all 4 are built.
               Uses can_transform() check before lookup so timer never blocks.
"""

import threading
import numpy as np
import cv2
import rclpy
from rclpy.node    import Node
from rclpy.time    import Time
from rclpy.duration import Duration
from nav_msgs.msg  import OccupancyGrid
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge     import CvBridge
from scipy.spatial.transform import Rotation
from tf2_ros import Buffer, TransformListener, TransformException

# -- Grid constants -----------------------------------------------------------
GRID_SIZE  = 300
RESOLUTION = 0.2          # m per cell
ORIGIN_X   = -20.0        # metres  (grid cell 0 = ORIGIN_X)
ORIGIN_Y   = -30.0
MAX_DIST   = 40.0         # ignore ground-ray hits beyond this distance

# -- LiDAR height thresholds (in base_link frame) -----------------------------
LIDAR_Z_MIN  = -0.30   # below this: noise / below ground, ignore
LIDAR_Z_CURB =  0.05   # flat ground below this -> free (0)
LIDAR_Z_OBS  =  0.15   # curb band 0.05-0.15m -> uncertain (50); above -> obstacle (100)
LIDAR_Z_POLE =  0.40   # pole shaft above this height -> solid obstacle
LIDAR_Z_MAX  =  3.50   # above this: ignore (tall structure, not relevant to 2D grid)

# -- Camera definitions -------------------------------------------------------
# (name, tf_frame, semantic_topic, camera_info_topic)
_CAMERAS = [
    ('front', 'hero/rgb_front', '/semantic/front', '/carla/hero/rgb_front/camera_info'),
    ('right', 'hero/rgb_right', '/semantic/right', '/carla/hero/rgb_right/camera_info'),
    ('back',  'hero/rgb_back',  '/semantic/back',  '/carla/hero/rgb_back/camera_info'),
    ('left',  'hero/rgb_left',  '/semantic/left',  '/carla/hero/rgb_left/camera_info'),
]

# -- Cityscapes RGB -> occupancy value ----------------------------------------
# Values: -1 = unknown/skip, 0 = free (drivable), 100 = obstacle
# Intermediate values signal varying hazard levels to the costmap.
_COLOR_OCC = {
    (128,  64, 128):   0,   # road           -> free / fully drivable
    (244,  35, 232):  80,   # sidewalk        -> non-drivable (curb boundary)
    ( 70,  70,  70): 100,   # building        -> hard obstacle
    (102, 102, 156): 100,   # wall            -> hard obstacle
    (190, 153, 153): 100,   # fence           -> hard obstacle
    (153, 153, 153):  50,   # pole            -> uncertain (handled further in LiDAR path)
    (250, 170,  30):  50,   # traffic light   -> uncertain
    (220, 220,   0):  50,   # traffic sign    -> uncertain
    (107, 142,  35):  30,   # vegetation      -> mostly avoid (overgrown edge)
    (145, 170, 100):  10,   # terrain         -> low-hazard ground, passable
    ( 70, 130, 180):  -1,   # sky             -> skip (no ground projection)
    (220,  20,  60): 100,   # person          -> obstacle
    (255,   0,   0): 100,   # rider           -> obstacle
    (  0,   0, 142): 100,   # car             -> obstacle
    (  0,   0,  70): 100,   # truck           -> obstacle
    (  0,  60, 100): 100,   # bus             -> obstacle
    (  0,  80, 100): 100,   # train           -> obstacle
    (  0,   0, 230): 100,   # motorcycle      -> obstacle
    (119,  11,  32): 100,   # bicycle         -> obstacle
}

# Pole RGB (used in LiDAR semantic override path)
_POLE_RGB = (153, 153, 153)

def _build_color_lut():
    """Build a 256x256x256 lookup table: RGB byte -> occupancy int8."""
    lut = np.full((256, 256, 256), -1, dtype=np.int8)
    for (r, g, b), occ in _COLOR_OCC.items():
        lut[r, g, b] = occ
    return lut

_COLOR_LUT = _build_color_lut()   # built once at import time

# Pre-compute pole occupancy LUT for fast camera-label check in LiDAR path
_POLE_OCC_VAL = np.int8(50)


class _CamLUT:
    """Pre-computed per-camera ground-plane LUT and projection matrices."""
    __slots__ = (
        'gp_gc', 'gp_gr', 'gp_valid',      # ground-plane projection
        't_base', 'R_cb', 'R_bc',           # extrinsics
        'K', 'K_inv',                        # intrinsics
        'img_h', 'img_w',
        'ray_base',                          # (H*W, 3) unit rays in base_link
    )


class HybridPerceptionGridNode(Node):

    def __init__(self):
        super().__init__('hybrid_perception_grid_node')
        self.bridge = CvBridge()

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._lock = threading.Lock()

        # Per-camera state
        self._K   = {}    # name -> 3x3 intrinsic matrix
        self._lut = {}    # name -> _CamLUT
        self._sem = {}    # name -> latest semantic image (H,W,3) uint8

        self.lidar_cloud = None   # latest PointCloud2 in base_link

        qos_be  = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=10)

        for name, frame, sem_t, info_t in _CAMERAS:
            self.create_subscription(CameraInfo, info_t,
                lambda m, n=name: self._cb_info(m, n), qos_rel)
            self.create_subscription(Image, sem_t,
                lambda m, n=name: self._cb_sem(m, n), qos_be)

        self.create_subscription(PointCloud2, '/lidar/filtered',
            self._cb_lidar, qos_be)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/occupancy/current', 10)

        # 1-Hz LUT rebuild timer -- retries until all cameras have good LUTs
        self.create_timer(1.0, self._lut_timer)
        # 20-Hz publish timer (matches route_costmap_node/ACC control rate)
        self.create_timer(0.05, self._publish_loop)

        self.get_logger().info('HybridPerceptionGridNode ready (4-camera + segmented LiDAR).')

    # -- Callbacks ------------------------------------------------------------

    def _cb_info(self, msg: CameraInfo, name: str):
        if name in self._K:
            return
        self._K[name] = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.get_logger().info(f'[{name}] camera_info received, K stored.')

    def _cb_sem(self, msg: Image, name: str):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self._lock:
            self._sem[name] = img

    def _cb_lidar(self, msg: PointCloud2):
        with self._lock:
            self.lidar_cloud = msg

    # -- LUT rebuild timer ----------------------------------------------------

    def _lut_timer(self):
        """Fires every second. Tries to build LUT for any camera missing one."""
        for name, frame, _, _ in _CAMERAS:
            if name not in self._K:
                continue
            already_good = (name in self._lut) and self._lut[name].gp_valid.any()
            if already_good:
                continue
            self._try_build_lut(name, frame)

    def _try_build_lut(self, name: str, cam_frame: str):
        if name not in self._K:
            return

        # Non-blocking TF check
        if not self.tf_buffer.can_transform('base_link', cam_frame, Time()):
            self.get_logger().warn(
                f'[{name}] TF base_link->{cam_frame} not yet available.',
                throttle_duration_sec=5.0)
            return

        try:
            tf = self.tf_buffer.lookup_transform('base_link', cam_frame, Time())
        except TransformException as ex:
            self.get_logger().warn(f'[{name}] TF lookup failed: {ex}',
                                   throttle_duration_sec=5.0)
            return

        K   = self._K[name]
        q   = tf.transform.rotation
        tr  = tf.transform.translation

        # R_cb: rotation from camera frame to base_link
        R_cb = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix().astype(np.float64)
        R_bc = R_cb.T                                           # base_link -> camera
        t_base = np.array([tr.x, tr.y, tr.z], dtype=np.float64)  # cam origin in base_link

        # Use latest semantic image dims if available, else default
        sem = self._sem.get(name)
        H, W = sem.shape[:2] if sem is not None else (600, 800)

        # Build pixel-ray grid
        K_inv = np.linalg.inv(K)
        us = np.arange(W, dtype=np.float64)
        vs = np.arange(H, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)
        uvh      = np.stack([uu.ravel(), vv.ravel(), np.ones(H * W)])  # (3, H*W)
        rays_cam = (K_inv @ uvh).T                                      # (H*W, 3)
        ray_base = (R_cb @ rays_cam.T).T.astype(np.float32)            # (H*W, 3)

        # Ground-plane intersection: z = 0  ->  t_base[2] + lam*ray_z = 0
        ray_z    = ray_base[:, 2]
        safe     = np.abs(ray_z) > 1e-6
        lam      = np.where(safe, -t_base[2] / ray_z, 0.0)
        pts      = t_base + lam[:, np.newaxis] * ray_base

        gc = ((pts[:, 0] - ORIGIN_X) / RESOLUTION).astype(np.int32)
        gr = ((pts[:, 1] - ORIGIN_Y) / RESOLUTION).astype(np.int32)
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
        lut.t_base   = t_base.astype(np.float32)
        lut.R_cb     = R_cb.astype(np.float32)
        lut.R_bc     = R_bc.astype(np.float32)
        lut.K        = K.astype(np.float32)
        lut.K_inv    = K_inv.astype(np.float32)
        lut.img_h    = H
        lut.img_w    = W
        lut.ray_base = ray_base

        with self._lock:
            self._lut[name] = lut

        n_valid = gp_valid.sum()
        self.get_logger().info(
            f'[{name}] LUT built -- {n_valid}/{H*W} ground-plane pixels. '
            f'cam={cam_frame} t=[{t_base[0]:.2f},{t_base[1]:.2f},{t_base[2]:.2f}]')

    # -- Camera projection ----------------------------------------------------

    def _project_camera(self, sem: np.ndarray, lut: _CamLUT) -> np.ndarray:
        """Project one semantic image onto the BEV grid.
        Returns (GRID_SIZE, GRID_SIZE) int8 layer: values from _COLOR_LUT, -1 = unknown.
        """
        layer = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)

        if sem.shape[0] != lut.img_h or sem.shape[1] != lut.img_w:
            sem = cv2.resize(sem, (lut.img_w, lut.img_h), interpolation=cv2.INTER_NEAREST)

        # Vectorised colour->occupancy lookup
        flat = sem.reshape(-1, 3)                                   # (H*W, 3)
        occ_flat = _COLOR_LUT[flat[:, 0], flat[:, 1], flat[:, 2]]  # (H*W,) int8

        use = lut.gp_valid & (occ_flat != -1)
        if use.any():
            layer[lut.gp_gr[use], lut.gp_gc[use]] = occ_flat[use]

        return layer

    # -- Segmented LiDAR projection -------------------------------------------

    def _parse_cloud(self, msg: PointCloud2):
        """Return (N,3) float32 xyz array from a PointCloud2 in base_link."""
        n  = msg.width * msg.height
        ps = msg.point_step
        if n == 0:
            return np.empty((0, 3), dtype=np.float32)

        x_off = y_off = z_off = 0
        for f in msg.fields:
            if   f.name == 'x': x_off = f.offset
            elif f.name == 'y': y_off = f.offset
            elif f.name == 'z': z_off = f.offset

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)
        xs  = np.frombuffer(raw[:, x_off:x_off+4].tobytes(), dtype=np.float32)
        ys  = np.frombuffer(raw[:, y_off:y_off+4].tobytes(), dtype=np.float32)
        zs  = np.frombuffer(raw[:, z_off:z_off+4].tobytes(), dtype=np.float32)
        return np.stack([xs, ys, zs], axis=1)

    def _height_fallback(self, z: np.ndarray) -> np.ndarray:
        """3-tier height-based occupancy for LiDAR dead-zone points.

        Tier 1  z < LIDAR_Z_CURB (0.05m)  : flat ground -> free (0)
        Tier 2  0.05 <= z <= 0.15m         : curb-height band -> uncertain (50)
        Tier 3  z > LIDAR_Z_OBS  (0.15m)  : above-ground mass -> obstacle (100)
        """
        occ = np.where(z < LIDAR_Z_CURB,
                       np.int8(0),
               np.where(z <= LIDAR_Z_OBS,
                        np.int8(50),
                        np.int8(100))).astype(np.int8)
        return occ

    def _project_lidar(self, cloud_msg: PointCloud2,
                       sem_imgs: dict, luts: dict) -> np.ndarray:
        """
        Classify each LiDAR point:
          1. Project onto each camera image to get a semantic label.
          2. Apply pole-height correction: pole pixels above LIDAR_Z_POLE -> 100.
          3. Dead zone (no camera coverage) -> 3-tier height fallback.
        Returns (GRID_SIZE, GRID_SIZE) int8 layer.
        """
        layer = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)

        xyz = self._parse_cloud(cloud_msg)
        if len(xyz) == 0:
            return layer

        # Height filter: ignore points outside valid range
        z_ok = (xyz[:, 2] > LIDAR_Z_MIN) & (xyz[:, 2] < LIDAR_Z_MAX)
        xyz  = xyz[z_ok]
        if len(xyz) == 0:
            return layer

        N = len(xyz)
        # Start with height-based fallback for all points
        occ     = self._height_fallback(xyz[:, 2])
        labeled = np.zeros(N, dtype=bool)   # True once assigned a semantic label

        # Try to project each point onto a camera image for semantic label
        for name, lut in luts.items():
            sem = sem_imgs.get(name)
            if sem is None:
                continue

            # Transform xyz (base_link) -> camera frame
            pts_cam = (xyz - lut.t_base) @ lut.R_bc.T   # (N,3)

            # Only points in front of the camera (positive depth)
            in_front = pts_cam[:, 2] > 0.1
            if not in_front.any():
                continue

            # Project to pixel coords
            depth  = pts_cam[in_front, 2]
            pts_n  = pts_cam[in_front] / depth[:, np.newaxis]
            K      = lut.K
            u = (K[0, 0] * pts_n[:, 0] + K[0, 2]).astype(np.int32)
            v = (K[1, 1] * pts_n[:, 1] + K[1, 2]).astype(np.int32)

            H, W = sem.shape[:2]
            if sem.shape[0] != lut.img_h or sem.shape[1] != lut.img_w:
                sem = cv2.resize(sem, (lut.img_w, lut.img_h),
                                 interpolation=cv2.INTER_NEAREST)
                H, W = lut.img_h, lut.img_w

            in_img = (u >= 0) & (u < W) & (v >= 0) & (v < H)

            idx_full = np.where(in_front)[0][in_img]
            sem_px   = sem[v[in_img], u[in_img]]           # (M,3)
            cam_occ  = _COLOR_LUT[sem_px[:, 0], sem_px[:, 1], sem_px[:, 2]]

            # Only override if semantic label is meaningful (>= 0)
            valid_label = cam_occ >= 0
            target_idx  = idx_full[valid_label]
            cam_vals    = cam_occ[valid_label]

            # Pole height correction:
            # A camera-labeled pole at height > LIDAR_Z_POLE is the pole shaft -> hard obstacle.
            # At or below LIDAR_Z_POLE it is the base / mounting -> keep camera value (50).
            is_pole = (sem_px[valid_label, 0] == _POLE_RGB[0]) & \
                      (sem_px[valid_label, 1] == _POLE_RGB[1]) & \
                      (sem_px[valid_label, 2] == _POLE_RGB[2])
            pole_shaft = is_pole & (xyz[target_idx, 2] > LIDAR_Z_POLE)
            cam_vals   = cam_vals.copy()
            cam_vals[pole_shaft] = np.int8(100)

            occ[target_idx]     = cam_vals
            labeled[target_idx] = True

        # Map every LiDAR point to its grid cell
        gc = ((xyz[:, 0] - ORIGIN_X) / RESOLUTION).astype(np.int32)
        gr = ((xyz[:, 1] - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        in_grid = (gc >= 0) & (gc < GRID_SIZE) & (gr >= 0) & (gr < GRID_SIZE)

        layer[gr[in_grid], gc[in_grid]] = occ[in_grid]
        return layer

    # -- Publish loop ---------------------------------------------------------

    def _publish_loop(self):
        with self._lock:
            sem_snap   = dict(self._sem)
            lut_snap   = dict(self._lut)
            cloud_snap = self.lidar_cloud

        grid = np.full((GRID_SIZE, GRID_SIZE), -1, dtype=np.int8)

        # === Camera layers ===
        cam_coverage = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)
        for name, _, _, _ in _CAMERAS:
            if name not in lut_snap or name not in sem_snap:
                continue
            layer = self._project_camera(sem_snap[name], lut_snap[name])
            known = layer >= 0
            # Camera overwrites unknown cells; blend where multiple cameras overlap
            blend = known & cam_coverage
            grid[known & ~cam_coverage] = layer[known & ~cam_coverage]
            if blend.any():
                grid[blend] = ((grid[blend].astype(np.int16) +
                                layer[blend].astype(np.int16)) // 2).astype(np.int8)
            cam_coverage |= known

        # === LiDAR layer (semantic + dead-zone fill) ===
        if cloud_snap is not None:
            lidar_layer = self._project_lidar(cloud_snap, sem_snap, lut_snap)
            lidar_known = lidar_layer >= 0

            # Fill cells not covered by any camera (dead zones)
            dead_zone = lidar_known & ~cam_coverage
            grid[dead_zone] = lidar_layer[dead_zone]

            # In camera-covered cells, blend: 70% camera / 30% LiDAR
            overlap = lidar_known & cam_coverage
            if overlap.any():
                blended = (0.7 * grid[overlap].astype(np.float32) +
                           0.3 * lidar_layer[overlap].astype(np.float32))
                grid[overlap] = np.clip(blended, 0, 100).astype(np.int8)

        # Publish
        msg                         = OccupancyGrid()
        msg.header.stamp            = self.get_clock().now().to_msg()
        msg.header.frame_id         = 'base_link'
        msg.info.resolution         = RESOLUTION
        msg.info.width              = GRID_SIZE
        msg.info.height             = GRID_SIZE
        msg.info.origin.position.x  = ORIGIN_X
        msg.info.origin.position.y  = ORIGIN_Y
        msg.info.origin.position.z  = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data                    = grid.ravel().tolist()
        try:
            self.pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'Publish failed (transient): {e}', throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = HybridPerceptionGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
