#!/usr/bin/env python3
"""
perception_drivable_grid_node.py  —  Camera-first real-time drivable area grid

Answers "can we drive here RIGHT NOW?" for every 20×20 cm patch around the car
using live sensor observations only.  Does NOT depend on HD maps or any
pre-built model of the environment.

Root cause fix vs. previous version
────────────────────────────────────
The old node waited for TF transforms of camera frames that do not exist in
the TF tree (the CARLA bridge publishes hero→base_link only; no camera
child frames).  LUTs never built → evidence grid stayed at 0.5 → all gray.
Fix: hardcode camera extrinsics from carla_objects.json; build LUTs at init.

Pipeline  (10 Hz)
─────────────────
  1. Project PSPNet semantic images → per-cell road observations via
     pre-built pixel→grid LUT (built once at startup, no TF needed)
  2. Cross-validate with LiDAR height: override camera "road" if any
     obstacle point sits > 5 cm above the assumed ground plane
  3. Temporal accumulation with odometry-based pose compensation
     (cv2.warpAffine on evidence grid each odom tick)
  4. Gaussian boundary smoothing on evidence grid
  5. Intersection handling: cos² corridor weighting when road area > threshold
  6. Morphological cleanup: open+close with 3×3 ellipse kernel

Cameras (hardcoded from docker/carla-ros-bridge/launch/carla_objects.json)
─────────────────────────────────────────────────────────────────────────────
  K  :  fx = fy = 571.12,  cx = 400,  cy = 300  (fov=70°, 800×600)

  R_cb(θ)  camera→base_link rotation for CARLA yaw θ:
       | sin θ    0   cos θ |
       |−cos θ    0   sin θ |
       |   0     −1     0   |

  Camera    CARLA pos (x,y,z)   yaw     Semantic topic
  ─────────────────────────────────────────────────────
  front     (0.7, −0.15, 1.88)   0°    /semantic/front
  right     (0.7, −0.15, 1.88) −70°    /semantic/right
  left      (0.7,  0.15, 1.88) +70°    /semantic/left
  (back camera not present in carla_objects.json)

Output
──────
  /grid/drivable/segmented  —  300×300 OccupancyGrid, 0.2 m/cell, base_link
      0   = confirmed drivable
    100   = obstacle / non-drivable
     50   = uncertain (insufficient observations yet)
"""

import math
import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2

# ── Grid constants ─────────────────────────────────────────────────────────────
GRID_SIZE   = 300
RESOLUTION  = 0.2     # metres per cell
ORIGIN_X    = -20.0   # base_link x of grid col 0
ORIGIN_Y    = -30.0   # base_link y of grid row 0

MAX_PROJ_DIST = 40.0  # ignore ground projections beyond this radius

# ── LiDAR height thresholds ────────────────────────────────────────────────────
LIDAR_Z_MIN  = -0.50   # below this = below sensor mount, skip
LIDAR_Z_FLAT =  0.05   # below this = flat ground (confirms drivable)
LIDAR_Z_CURB =  0.15   # below this = curb (uncertain)
LIDAR_Z_OBS  =  0.30   # above this = clear obstacle (override camera road)
LIDAR_Z_MAX  =  3.50   # above this = too high (building, bridge), skip

# ── Temporal accumulation ──────────────────────────────────────────────────────
ALPHA_BLEND   = 0.65   # weight of running average (vs. new observation)
DECAY_RATE    = 0.992  # per-cycle decay for unobserved cells → 0.5
INIT_EVIDENCE = 0.50   # starting value: uncertain

# ── Camera intrinsics (fov=70°, 800×600) ──────────────────────────────────────
_FX = _FY = 571.12
_CX, _CY   = 400.0, 300.0
_IMG_W, _IMG_H = 800, 600

_K = np.array([
    [_FX,  0.0, _CX],
    [0.0, _FY,  _CY],
    [0.0,  0.0,  1.0],
], dtype=np.float64)
_K_INV = np.linalg.inv(_K)


def _R_cb(yaw_deg: float) -> np.ndarray:
    """Camera-frame → base_link rotation for CARLA yaw (degrees)."""
    t = math.radians(yaw_deg)
    s, c = math.sin(t), math.cos(t)
    return np.array([
        [ s,  0,  c],
        [-c,  0,  s],
        [ 0, -1,  0],
    ], dtype=np.float64)


# (name, semantic_topic, t_base [x,y,z in base_link], R_cb)
_CAMERAS_HC = [
    ('front', '/semantic/front',
     np.array([0.70, -0.15, 1.88], dtype=np.float64), _R_cb(0.0)),
    ('right', '/semantic/right',
     np.array([0.70, -0.15, 1.88], dtype=np.float64), _R_cb(-70.0)),
    ('left',  '/semantic/left',
     np.array([0.70,  0.15, 1.88], dtype=np.float64), _R_cb(70.0)),
]

# ── Cityscapes RGB → observation score ─────────────────────────────────────────
#   score [0,100]:  100 = strongly drivable, 0 = obstacle/non-drivable
#   -1 = skip pixel entirely (sky, unknown)
_COLOR_OBS_MAP = {
    (128,  64, 128): 100,   # road
    (244,  35, 232):   5,   # sidewalk     (not drivable for car)
    ( 70,  70,  70):   0,   # building
    (102, 102, 156):   0,   # wall
    (190, 153, 153):   0,   # fence
    (153, 153, 153):  20,   # pole
    (250, 170,  30):  20,   # traffic light
    (220, 220,   0):  20,   # traffic sign
    (107, 142,  35):  10,   # vegetation
    (145, 170, 100):  60,   # terrain
    ( 70, 130, 180):  -1,   # sky          (skip)
    (220,  20,  60):   0,   # person
    (255,   0,   0):   0,   # rider
    (  0,   0, 142):   0,   # car
    (  0,   0,  70):   0,   # truck
    (  0,  60, 100):   0,   # bus
    (  0,  80, 100):   0,   # train
    (  0,   0, 230):   0,   # motorcycle
    (119,  11,  32):   0,   # bicycle
}


def _build_obs_lut() -> np.ndarray:
    """Build a 256³ lookup table: lut[R,G,B] → score (int16, -1=skip)."""
    lut = np.full((256, 256, 256), -1, dtype=np.int16)
    for (r, g, b), v in _COLOR_OBS_MAP.items():
        lut[r, g, b] = v
    return lut


_OBS_LUT = _build_obs_lut()


# ── Per-camera ground-plane LUT ────────────────────────────────────────────────

class _CamLUT:
    """Pre-computed ground-plane projection for one camera."""
    __slots__ = ('gc', 'gr', 'valid', 'img_h', 'img_w')

    def __init__(self, t_base: np.ndarray, R_cb: np.ndarray,
                 img_h: int = _IMG_H, img_w: int = _IMG_W):
        # All pixel coords as flat arrays
        us = np.arange(img_w, dtype=np.float64)
        vs = np.arange(img_h, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)                        # (H, W)
        uvh = np.stack([uu.ravel(), vv.ravel(),
                        np.ones(img_h * img_w)])             # (3, H*W)

        rays_cam  = (_K_INV @ uvh).T                        # (N, 3)  camera frame
        ray_base  = (R_cb @ rays_cam.T).T.astype(np.float32) # (N, 3) base_link

        # Ground-plane intersection: z=0 in base_link
        # P(λ) = t_base + λ * ray_base;  z: 0 = t_base[2] + λ*ray_base[:,2]
        rz   = ray_base[:, 2]
        safe = np.abs(rz) > 1e-6
        lam  = np.where(safe, -t_base[2] / rz, 0.0)

        px = t_base[0] + lam * ray_base[:, 0]
        py = t_base[1] + lam * ray_base[:, 1]

        self.gc = ((px - ORIGIN_X) / RESOLUTION).astype(np.int32)
        self.gr = ((py - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        d2d     = np.hypot(px, py)

        self.valid = (
            safe
            & (lam > 0.0)                          # ray goes toward the ground
            & (d2d < MAX_PROJ_DIST)                # not too far away
            & (self.gc >= 0) & (self.gc < GRID_SIZE)
            & (self.gr >= 0) & (self.gr < GRID_SIZE)
        )
        self.img_h = img_h
        self.img_w = img_w


# ── Main node ──────────────────────────────────────────────────────────────────

class PerceptionDrivableGridNode(Node):

    def __init__(self):
        super().__init__('perception_drivable_grid_node')
        self.bridge = CvBridge()
        self._lock  = threading.Lock()

        # ── Build LUTs immediately (no TF needed) ─────────────────────────────
        self._luts: dict[str, _CamLUT] = {}
        for name, _topic, t_base, R_cb in _CAMERAS_HC:
            self._luts[name] = _CamLUT(t_base, R_cb)
            n_valid = int(self._luts[name].valid.sum())
            self.get_logger().info(
                f'[{name}] LUT ready — {n_valid}/{_IMG_H * _IMG_W} '
                f'ground-plane pixels ({100*n_valid/(_IMG_H*_IMG_W):.1f}%)')

        # ── Evidence grid ──────────────────────────────────────────────────────
        self._evidence = np.full((GRID_SIZE, GRID_SIZE),
                                 INIT_EVIDENCE, dtype=np.float32)

        # ── Latest sensor snapshots ────────────────────────────────────────────
        self._sem: dict[str, np.ndarray] = {}   # name → RGB uint8
        self._lidar_cloud = None                # PointCloud2
        self._last_x: float   = None
        self._last_y: float   = None
        self._last_yaw: float = None

        # ── QoS ───────────────────────────────────────────────────────────────
        qos_be  = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)
        qos_rel = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=5)

        # ── Subscriptions ──────────────────────────────────────────────────────
        for name, topic, _t, _R in _CAMERAS_HC:
            self.create_subscription(
                Image, topic,
                lambda m, n=name: self._cb_sem(m, n),
                qos_be)

        self.create_subscription(
            PointCloud2, '/lidar/filtered',
            self._cb_lidar, qos_be)
        self.create_subscription(
            Odometry, '/gnss/odometry',
            self._cb_odom, qos_be)

        # ── Publisher ─────────────────────────────────────────────────────────
        self.pub = self.create_publisher(
            OccupancyGrid, '/grid/drivable/segmented', 10)

        # ── 10 Hz publish loop ────────────────────────────────────────────────
        self.create_timer(0.1, self._publish_loop)

        self.get_logger().info(
            'PerceptionDrivableGridNode ready — publishing /grid/drivable/segmented')

    # ── Sensor callbacks ───────────────────────────────────────────────────────

    def _cb_sem(self, msg: Image, name: str):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self._lock:
            self._sem[name] = img

    def _cb_lidar(self, msg: PointCloud2):
        with self._lock:
            self._lidar_cloud = msg

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        x, y = p.x, p.y
        _, _, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        with self._lock:
            if self._last_x is None:
                self._last_x, self._last_y, self._last_yaw = x, y, yaw
                return
            self._compensate_pose(x, y, yaw)
            self._last_x, self._last_y, self._last_yaw = x, y, yaw

    # ── Step 1 : Camera → observation grid ────────────────────────────────────

    def _camera_obs_grid(self, sem_snap: dict):
        """Project all camera semantic images onto the BEV grid.

        Returns (obs_sum, obs_count) each (GRID_SIZE, GRID_SIZE) float32.
        Mean score = obs_sum / obs_count (where obs_count > 0).
        """
        obs_sum   = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        obs_count = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)

        for name, _topic, _t, _R in _CAMERAS_HC:
            sem = sem_snap.get(name)
            if sem is None:
                continue

            lut = self._luts[name]

            # Resize if needed (should be no-op at 800×600)
            if sem.shape[0] != lut.img_h or sem.shape[1] != lut.img_w:
                sem = cv2.resize(sem, (lut.img_w, lut.img_h),
                                 interpolation=cv2.INTER_NEAREST)

            flat   = sem.reshape(-1, 3)                         # (N, 3)
            scores = _OBS_LUT[flat[:, 0], flat[:, 1], flat[:, 2]]  # int16

            use = lut.valid & (scores >= 0)
            if not use.any():
                continue

            np.add.at(obs_sum,   (lut.gr[use], lut.gc[use]),
                      scores[use].astype(np.float32))
            np.add.at(obs_count, (lut.gr[use], lut.gc[use]), 1.0)

        return obs_sum, obs_count

    # ── Step 2 : LiDAR height grid ────────────────────────────────────────────

    def _lidar_height_grid(self, msg: PointCloud2) -> np.ndarray:
        """Extract per-cell max height from PointCloud2.

        Returns (GRID_SIZE, GRID_SIZE) float32; NaN = no coverage.
        """
        hg = np.full((GRID_SIZE, GRID_SIZE), np.nan, dtype=np.float32)
        n  = msg.width * msg.height
        if n == 0:
            return hg

        ps = msg.point_step
        x_off = y_off = z_off = 0
        for f in msg.fields:
            if   f.name == 'x': x_off = f.offset
            elif f.name == 'y': y_off = f.offset
            elif f.name == 'z': z_off = f.offset

        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)
        xs  = np.frombuffer(raw[:, x_off:x_off+4].tobytes(), dtype=np.float32)
        ys  = np.frombuffer(raw[:, y_off:y_off+4].tobytes(), dtype=np.float32)
        zs  = np.frombuffer(raw[:, z_off:z_off+4].tobytes(), dtype=np.float32)

        valid = np.isfinite(xs) & np.isfinite(zs) & (zs > LIDAR_Z_MIN) & (zs < LIDAR_Z_MAX)
        xs, ys, zs = xs[valid], ys[valid], zs[valid]
        if len(zs) == 0:
            return hg

        gc = ((xs - ORIGIN_X) / RESOLUTION).astype(np.int32)
        gr = ((ys - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        ing = (gc >= 0) & (gc < GRID_SIZE) & (gr >= 0) & (gr < GRID_SIZE)
        gc, gr, zs = gc[ing], gr[ing], zs[ing]

        # max height per cell (obstacle = tall point)
        np.maximum.at(hg, (gr, gc), zs)
        return hg

    def _fuse_camera_lidar(self,
                           obs_sum: np.ndarray, obs_count: np.ndarray,
                           hg: np.ndarray):
        """Produce per-cell frame observation score [0,1] and observed mask.

        Camera is primary; LiDAR overrides when it detects an obstacle.
        """
        frame_obs = np.full((GRID_SIZE, GRID_SIZE), -1.0, dtype=np.float32)
        observed  = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)

        cam_seen = obs_count > 0
        frame_obs[cam_seen] = obs_sum[cam_seen] / (obs_count[cam_seen] * 100.0)
        observed[cam_seen]  = True

        lidar_has = np.isfinite(hg)
        cam_road  = cam_seen & (frame_obs >= 0.70)   # camera confident it's road

        # Camera says road but LiDAR sees obstacle above threshold → override
        lidar_block = cam_road & lidar_has & (hg > LIDAR_Z_OBS)
        frame_obs[lidar_block] = 0.0

        # Camera says road and LiDAR confirms flat → slight confidence boost
        lidar_flat = cam_road & lidar_has & (hg < LIDAR_Z_FLAT)
        frame_obs[lidar_flat] = np.minimum(1.0, frame_obs[lidar_flat] * 1.05)

        # LiDAR-only zones (no camera coverage): use height to estimate drivability
        lidar_only = lidar_has & ~cam_seen
        z = hg[lidar_only]
        frame_obs[lidar_only] = np.where(
            z < LIDAR_Z_FLAT, 1.0,
            np.where(z < LIDAR_Z_CURB, 0.5, 0.0))
        observed[lidar_only] = True

        return frame_obs, observed

    # ── Step 3 : Pose compensation + temporal blend ───────────────────────────

    def _compensate_pose(self, x: float, y: float, yaw: float):
        """Shift evidence grid by vehicle delta pose.  Caller must hold _lock."""
        dx   = x   - self._last_x
        dy   = y   - self._last_y
        dyaw = yaw - self._last_yaw
        # Wrap yaw delta
        dyaw = (dyaw + math.pi) % (2 * math.pi) - math.pi

        # Shift amounts in grid cells
        shift_col = -dx / RESOLUTION
        shift_row = -dy / RESOLUTION

        # Rotation centre = vehicle position in grid
        vc = (0.0 - ORIGIN_X) / RESOLUTION
        vr = (0.0 - ORIGIN_Y) / RESOLUTION

        ca, sa = math.cos(-dyaw), math.sin(-dyaw)
        M = np.float32([
            [ca, -sa, shift_col + vc*(1-ca) + vr*sa],
            [sa,  ca, shift_row - vc*sa     + vr*(1-ca)],
        ])
        self._evidence = cv2.warpAffine(
            self._evidence, M, (GRID_SIZE, GRID_SIZE),
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=INIT_EVIDENCE)

    def _blend_evidence(self, frame_obs: np.ndarray, observed: np.ndarray):
        """Alpha-blend new observations into evidence.  Caller must hold _lock."""
        # Unobserved cells decay toward uncertain
        un = ~observed
        self._evidence[un] = (DECAY_RATE * self._evidence[un]
                              + (1.0 - DECAY_RATE) * INIT_EVIDENCE)
        # Observed cells: weighted average
        e = self._evidence[observed]
        o = frame_obs[observed]
        self._evidence[observed] = ALPHA_BLEND * e + (1.0 - ALPHA_BLEND) * o
        np.clip(self._evidence, 0.0, 1.0, out=self._evidence)

    # ── Step 4 : Boundary smoothing ───────────────────────────────────────────

    @staticmethod
    def _smooth_boundaries(ev: np.ndarray) -> np.ndarray:
        return cv2.GaussianBlur(ev, (0, 0), sigmaX=1.2).astype(np.float32)

    # ── Step 5 : Intersection corridor ───────────────────────────────────────

    @staticmethod
    def _intersection_corridor(ev: np.ndarray) -> np.ndarray:
        """Suppress off-axis cells when road area is very large (intersection).

        In base_link the vehicle always points +x.  Weight cells by
        cos²(angle_from_forward) so cross streets are softly down-weighted.
        Activates only when road cell count > 2000 (~80 m²).
        """
        if int((ev > 0.55).sum()) < 2000:
            return ev

        rows = np.arange(GRID_SIZE, dtype=np.float32)
        cols = np.arange(GRID_SIZE, dtype=np.float32)
        cc, rr = np.meshgrid(cols, rows)
        bx = cc * RESOLUTION + ORIGIN_X
        by = rr * RESOLUTION + ORIGIN_Y
        ang    = np.arctan2(by, bx)
        weight = np.where(ev > 0.55, 0.60 + 0.40 * np.cos(ang) ** 2, 1.0)
        return (ev * weight).astype(np.float32)

    # ── Step 6 : Morphological cleanup ───────────────────────────────────────

    @staticmethod
    def _morpho_cleanup(ev: np.ndarray) -> np.ndarray:
        binary  = (ev > 0.55).astype(np.uint8) * 255
        kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
        out = ev.copy()
        out[(binary > 0) & (cleaned == 0)] = np.minimum(
            out[(binary > 0) & (cleaned == 0)], 0.45)
        out[(binary == 0) & (cleaned > 0)] = np.maximum(
            out[(binary == 0) & (cleaned > 0)], 0.60)
        return out

    # ── Publish loop ──────────────────────────────────────────────────────────

    def _publish_loop(self):
        with self._lock:
            sem_snap   = dict(self._sem)
            cloud_snap = self._lidar_cloud

        if not sem_snap:
            # No semantic images yet — publish uncertain grid so topic is visible
            pass

        # ── Steps 1+2 ─────────────────────────────────────────────────────────
        obs_sum, obs_count = self._camera_obs_grid(sem_snap)

        if cloud_snap is not None:
            hg = self._lidar_height_grid(cloud_snap)
        else:
            hg = np.full((GRID_SIZE, GRID_SIZE), np.nan, dtype=np.float32)

        frame_obs, observed = self._fuse_camera_lidar(obs_sum, obs_count, hg)

        # ── Step 3 ─────────────────────────────────────────────────────────────
        with self._lock:
            self._blend_evidence(frame_obs, observed)
            ev = self._evidence.copy()

        # ── Steps 4-6 (read-only on local copy, no lock needed) ───────────────
        ev = self._smooth_boundaries(ev)
        ev = self._intersection_corridor(ev)
        ev = self._morpho_cleanup(ev)

        # ── Encode: evidence 1.0=drivable→0, evidence 0.0=obstacle→100 ────────
        grid_out = np.clip(np.round((1.0 - ev) * 100.0), 0, 100).astype(np.int8)

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
        msg.data                      = grid_out.ravel().tolist()

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionDrivableGridNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
