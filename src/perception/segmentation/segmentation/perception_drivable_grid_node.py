#!/usr/bin/env python3
"""
perception_drivable_grid_node.py  —  Camera-first real-time drivable area grid

Pipeline  (10 Hz)
  1. Project PSPNet semantic images -> per-cell road observations (LUT at init)
  2. Cross-validate with LiDAR ground/obstacle evidence
  3. Temporal accumulation with odometry pose compensation
  4. Vehicle footprint prior (always drivable)
  5. Driven path prior (last N odom positions = drivable)
  6. Gaussian boundary smoothing
  7. Road dilation into blind spots (expand road into uncertain, not obstacle)
  8. LiDAR ground fill (uncertain cells confirmed by LiDAR ground returns -> drivable)
  9. Connected road fill (flood fill from vehicle footprint through all non-obstacle cells)
 10. Road corridor fill (column-wise span fill — cleans speckles, intersection-safe by geometry)
 11. Morphological cleanup

Cameras (hardcoded from carla_objects.json):
  K: fx=fy=571.12, cx=400, cy=300  (fov=70, 800x600)
  R_cb(theta) = [[sin t, 0, cos t], [-cos t, 0, sin t], [0, -1, 0]]
  front: t=(0.7,-0.15,1.88) yaw=0    /semantic/front
  right: t=(0.7,-0.15,1.88) yaw=-70  /semantic/right
  left:  t=(0.7, 0.15,1.88) yaw=+70  /semantic/left
  back:  t=(-1.5,0.0, 1.88) yaw=180  /semantic/back (needs rgb_back in carla_objects.json)

Output: /grid/drivable/segmented  300x300 OccupancyGrid 0.2m/cell base_link
"""

import math
import threading
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from scipy.ndimage import label as sp_label
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image, PointCloud2

GRID_SIZE   = 300
RESOLUTION  = 0.2
ORIGIN_X    = -20.0
ORIGIN_Y    = -30.0
MAX_PROJ_DIST = 40.0

# LiDAR evidence thresholds — count-based, not max-height
LIDAR_GROUND_Z     =  0.35   # road surface + 35 cm tolerance for slope/noise
LIDAR_OBS_Z        =  1.20   # true obstacle (building wall, vehicle body)
LIDAR_GND_MIN_HITS =  2      # need >= 2 ground returns to call cell "road"
LIDAR_GND_DRIVABLE =  0.85   # evidence value for LiDAR-confirmed road cells
LIDAR_GND_BOOST    =  0.72   # boost for uncertain camera cells with LiDAR ground support

# Connected-component fill thresholds
CC_TRAVERSABLE_MIN = 0.22   # cells with ev > this are "not obstacle" for flood fill
CC_UNCERTAIN_LO    = 0.35   # flood fill only boosts cells in this uncertain band
CC_UNCERTAIN_HI    = 0.65
CC_FILL_VALUE      = 0.70   # evidence assigned to flood-fill-reached uncertain cells

ALPHA_BLEND   = 0.65
DECAY_RATE    = 0.992
INIT_EVIDENCE = 0.50

PATH_HISTORY_MAXLEN = 50   # ~12 s at 4 Hz odom = 800 bytes

_VC = int((0.0 - ORIGIN_X) / RESOLUTION)   # 100  vehicle column
_VR = int((0.0 - ORIGIN_Y) / RESOLUTION)   # 150  vehicle row

FOOTPRINT_HALF_W = 10   # +/-2.0 m lateral
FOOTPRINT_REAR   =  5   # 1.0 m behind
FOOTPRINT_FRONT  = 25   # 5.0 m ahead
PATH_STAMP_HALF  =  8   # +/-1.6 m around each history point

PRIOR_VEHICLE  = 0.90
PRIOR_PATH     = 0.82
PRIOR_DILATION = 0.63

_FX = _FY = 571.12
_CX, _CY   = 400.0, 300.0
_IMG_W, _IMG_H = 800, 600

_K = np.array([[_FX, 0.0, _CX], [0.0, _FY, _CY], [0.0, 0.0, 1.0]], dtype=np.float64)
_K_INV = np.linalg.inv(_K)


def _R_cb(yaw_deg):
    t = math.radians(yaw_deg)
    s, c = math.sin(t), math.cos(t)
    return np.array([[s, 0, c], [-c, 0, s], [0, -1, 0]], dtype=np.float64)


_CAMERAS_HC = [
    ('front', '/semantic/front',  np.array([ 0.70, -0.15, 1.88]), _R_cb(  0.0)),
    ('right', '/semantic/right',  np.array([ 0.70, -0.15, 1.88]), _R_cb(-70.0)),
    ('left',  '/semantic/left',   np.array([ 0.70,  0.15, 1.88]), _R_cb( 70.0)),
    ('back',  '/semantic/back',   np.array([-1.50,  0.00, 1.88]), _R_cb(180.0)),
]

_COLOR_OBS_MAP = {
    (128,  64, 128): 100, (244,  35, 232):   5, ( 70,  70,  70):   0,
    (102, 102, 156):   0, (190, 153, 153):   0, (153, 153, 153):  20,
    (250, 170,  30):  20, (220, 220,   0):  20, (107, 142,  35):  10,
    (145, 170, 100):  60, ( 70, 130, 180):  -1, (220,  20,  60):   0,
    (255,   0,   0):   0, (  0,   0, 142):   0, (  0,   0,  70):   0,
    (  0,  60, 100):   0, (  0,  80, 100):   0, (  0,   0, 230):   0,
    (119,  11,  32):   0,
}


def _build_obs_lut():
    lut = np.full((256, 256, 256), -1, dtype=np.int16)
    for (r, g, b), v in _COLOR_OBS_MAP.items():
        lut[r, g, b] = v
    return lut


_OBS_LUT = _build_obs_lut()


class _CamLUT:
    __slots__ = ('gc', 'gr', 'valid', 'img_h', 'img_w')

    def __init__(self, t_base, R_cb, img_h=_IMG_H, img_w=_IMG_W):
        us, vs = np.arange(img_w, dtype=np.float64), np.arange(img_h, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)
        uvh = np.stack([uu.ravel(), vv.ravel(), np.ones(img_h * img_w)])
        rays_cam = (_K_INV @ uvh).T
        ray_base = (R_cb @ rays_cam.T).T.astype(np.float32)

        rz = ray_base[:, 2]
        safe = np.abs(rz) > 1e-6
        with np.errstate(divide='ignore', invalid='ignore'):
            lam = np.where(safe, -float(t_base[2]) / rz, 0.0)

        px = float(t_base[0]) + lam * ray_base[:, 0]
        py = float(t_base[1]) + lam * ray_base[:, 1]

        self.gc = ((px - ORIGIN_X) / RESOLUTION).astype(np.int32)
        self.gr = ((py - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        d2d = np.hypot(px, py)
        self.valid = (safe & (lam > 0.0) & (d2d < MAX_PROJ_DIST)
                      & (self.gc >= 0) & (self.gc < GRID_SIZE)
                      & (self.gr >= 0) & (self.gr < GRID_SIZE))
        self.img_h, self.img_w = img_h, img_w


class PerceptionDrivableGridNode(Node):

    def __init__(self):
        super().__init__('perception_drivable_grid_node')
        self.bridge = CvBridge()
        self._lock  = threading.Lock()

        self._luts = {}
        for name, _t, t_base, R_cb in _CAMERAS_HC:
            self._luts[name] = _CamLUT(t_base, R_cb)
            n = int(self._luts[name].valid.sum())
            self.get_logger().info(
                f'[{name}] LUT ready — {n}/{_IMG_H*_IMG_W} pixels ({100*n/(_IMG_H*_IMG_W):.1f}%)')

        self._evidence = np.full((GRID_SIZE, GRID_SIZE), INIT_EVIDENCE, dtype=np.float32)
        self._sem      = {}
        self._lidar_cloud = None

        self._last_x = self._last_y = self._last_yaw = None
        self._cur_x  = self._cur_y  = self._cur_yaw  = None
        self._path_history = deque(maxlen=PATH_HISTORY_MAXLEN)

        qos_be = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth=1)

        for name, topic, _t, _R in _CAMERAS_HC:
            self.create_subscription(Image, topic,
                lambda m, n=name: self._cb_sem(m, n), qos_be)

        self.create_subscription(PointCloud2, '/lidar/filtered', self._cb_lidar, qos_be)
        self.create_subscription(Odometry, '/gnss/odometry',    self._cb_odom,  qos_be)

        self.pub = self.create_publisher(OccupancyGrid, '/grid/drivable/segmented', 10)
        # 20-Hz (matches route_costmap_node/ACC control rate)
        self.create_timer(0.05, self._publish_loop)
        self.get_logger().info('PerceptionDrivableGridNode ready — /grid/drivable/segmented')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _cb_sem(self, msg, name):
        img = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        with self._lock:
            self._sem[name] = img

    def _cb_lidar(self, msg):
        with self._lock:
            self._lidar_cloud = msg

    def _cb_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        x, y = p.x, p.y
        _, _, yaw = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz')
        with self._lock:
            self._cur_x, self._cur_y, self._cur_yaw = x, y, yaw
            self._path_history.append((x, y))
            if self._last_x is None:
                self._last_x, self._last_y, self._last_yaw = x, y, yaw
                return
            self._compensate_pose(x, y, yaw)
            self._last_x, self._last_y, self._last_yaw = x, y, yaw

    # ── step 1: camera obs ────────────────────────────────────────────────────

    def _camera_obs_grid(self, sem_snap):
        obs_sum   = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        obs_count = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        for name, _t, _tb, _R in _CAMERAS_HC:
            sem = sem_snap.get(name)
            if sem is None:
                continue
            lut = self._luts[name]
            if sem.shape[0] != lut.img_h or sem.shape[1] != lut.img_w:
                sem = cv2.resize(sem, (lut.img_w, lut.img_h), interpolation=cv2.INTER_NEAREST)
            flat   = sem.reshape(-1, 3)
            scores = _OBS_LUT[flat[:, 0], flat[:, 1], flat[:, 2]]
            use = lut.valid & (scores >= 0)
            if not use.any():
                continue
            np.add.at(obs_sum,   (lut.gr[use], lut.gc[use]), scores[use].astype(np.float32))
            np.add.at(obs_count, (lut.gr[use], lut.gc[use]), 1.0)
        return obs_sum, obs_count

    # ── step 2: lidar evidence grid ───────────────────────────────────────────

    def _lidar_evidence_grid(self, msg):
        """Count ground hits (z < LIDAR_GROUND_Z) and obstacle hits (z >= LIDAR_OBS_Z)
        per grid cell.  Using counts rather than max-height avoids single-outlier
        returns from road slope or sensor noise marking a whole cell as obstacle."""
        gnd_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        obs_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
        n = msg.width * msg.height
        if n == 0:
            return gnd_cnt, obs_cnt
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
        ok  = np.isfinite(xs) & np.isfinite(zs)
        xs, ys, zs = xs[ok], ys[ok], zs[ok]
        if not len(zs):
            return gnd_cnt, obs_cnt
        gc = ((xs - ORIGIN_X) / RESOLUTION).astype(np.int32)
        gr = ((ys - ORIGIN_Y) / RESOLUTION).astype(np.int32)
        ing = (gc >= 0) & (gc < GRID_SIZE) & (gr >= 0) & (gr < GRID_SIZE)
        gnd_mask = ing & (zs >= -0.30) & (zs < LIDAR_GROUND_Z)
        obs_mask = ing & (zs >= LIDAR_OBS_Z)
        if gnd_mask.any():
            np.add.at(gnd_cnt, (gr[gnd_mask], gc[gnd_mask]), 1)
        if obs_mask.any():
            np.add.at(obs_cnt, (gr[obs_mask], gc[obs_mask]), 1)
        return gnd_cnt, obs_cnt

    def _fuse_camera_lidar(self, obs_sum, obs_count, gnd_cnt, obs_cnt):
        frame_obs = np.full((GRID_SIZE, GRID_SIZE), -1.0, dtype=np.float32)
        observed  = np.zeros((GRID_SIZE, GRID_SIZE), dtype=bool)

        cam_seen = obs_count > 0
        frame_obs[cam_seen] = obs_sum[cam_seen] / (obs_count[cam_seen] * 100.0)
        observed[cam_seen]  = True

        cam_road = cam_seen & (frame_obs >= 0.70)
        frame_obs[cam_road & (obs_cnt > 0)] = 0.0
        boost_mask = cam_road & (gnd_cnt >= LIDAR_GND_MIN_HITS) & (obs_cnt == 0)
        frame_obs[boost_mask] = np.minimum(1.0, frame_obs[boost_mask] * 1.05)

        has_ground = gnd_cnt >= LIDAR_GND_MIN_HITS
        has_obs    = obs_cnt > 0
        lidar_hit  = (gnd_cnt > 0) | (obs_cnt > 0)
        lidar_only = lidar_hit & ~cam_seen

        road_lidar = lidar_only & has_ground & ~has_obs
        frame_obs[road_lidar] = LIDAR_GND_DRIVABLE
        observed[road_lidar]  = True

        obs_lidar = lidar_only & has_obs
        frame_obs[obs_lidar] = 0.05
        observed[obs_lidar]  = True

        mixed_lidar = lidar_only & has_ground & has_obs
        frame_obs[mixed_lidar] = 0.45
        observed[mixed_lidar]  = True

        return frame_obs, observed

    # ── step 3: pose compensation + blend ─────────────────────────────────────

    def _compensate_pose(self, x, y, yaw):
        dx   = x   - self._last_x
        dy   = y   - self._last_y
        dyaw = (yaw - self._last_yaw + math.pi) % (2 * math.pi) - math.pi
        if abs(dx) < 0.02 and abs(dy) < 0.02 and abs(dyaw) < 0.008:
            return
        sc = -dx / RESOLUTION
        sr = -dy / RESOLUTION
        vc = (0.0 - ORIGIN_X) / RESOLUTION
        vr = (0.0 - ORIGIN_Y) / RESOLUTION
        ca, sa = math.cos(-dyaw), math.sin(-dyaw)
        M = np.float32([[ca, -sa, sc + vc*(1-ca) + vr*sa],
                        [sa,  ca, sr - vc*sa     + vr*(1-ca)]])
        self._evidence = cv2.warpAffine(self._evidence, M, (GRID_SIZE, GRID_SIZE),
            flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=INIT_EVIDENCE)

    def _blend_evidence(self, frame_obs, observed):
        un = ~observed
        self._evidence[un] = DECAY_RATE * self._evidence[un] + (1-DECAY_RATE) * INIT_EVIDENCE
        e, o = self._evidence[observed], frame_obs[observed]
        self._evidence[observed] = ALPHA_BLEND * e + (1 - ALPHA_BLEND) * o
        np.clip(self._evidence, 0.0, 1.0, out=self._evidence)

    # ── steps 4+5: vehicle footprint + path priors ────────────────────────────

    def _apply_priors(self):
        r0 = max(0, _VR - FOOTPRINT_HALF_W)
        r1 = min(GRID_SIZE, _VR + FOOTPRINT_HALF_W + 1)
        c0 = max(0, _VC - FOOTPRINT_REAR)
        c1 = min(GRID_SIZE, _VC + FOOTPRINT_FRONT + 1)
        self._evidence[r0:r1, c0:c1] = np.maximum(self._evidence[r0:r1, c0:c1], PRIOR_VEHICLE)

        if not self._path_history or self._cur_x is None:
            return

        cx, cy, cyaw = self._cur_x, self._cur_y, self._cur_yaw
        ca = math.cos(-cyaw)
        sa = math.sin(-cyaw)

        for (wx, wy) in self._path_history:
            dx, dy = wx - cx, wy - cy
            lx = ca * dx - sa * dy
            ly = sa * dx + ca * dy
            gc = int((lx - ORIGIN_X) / RESOLUTION)
            gr = int((ly - ORIGIN_Y) / RESOLUTION)
            if not (PATH_STAMP_HALF <= gc < GRID_SIZE - PATH_STAMP_HALF and
                    PATH_STAMP_HALF <= gr < GRID_SIZE - PATH_STAMP_HALF):
                continue
            r0 = gr - PATH_STAMP_HALF
            r1 = gr + PATH_STAMP_HALF + 1
            c0 = gc - PATH_STAMP_HALF
            c1 = gc + PATH_STAMP_HALF + 1
            self._evidence[r0:r1, c0:c1] = np.maximum(
                self._evidence[r0:r1, c0:c1], PRIOR_PATH)

    # ── step 6: smoothing ─────────────────────────────────────────────────────

    @staticmethod
    def _smooth_boundaries(ev):
        return cv2.GaussianBlur(ev, (0, 0), sigmaX=1.2).astype(np.float32)

    # ── step 7: road dilation into blind spots ────────────────────────────────

    @staticmethod
    def _dilate_into_blindspots(ev):
        road_mask  = (ev > 0.65).astype(np.uint8)
        uncertain  = (ev >= 0.36) & (ev <= 0.64)
        road_cells = int(road_mask.sum())
        ksize = 51 if road_cells > 1500 else 25
        kernel  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        dilated = cv2.dilate(road_mask, kernel).astype(bool)
        fill    = dilated & uncertain
        ev_out  = ev.copy()
        ev_out[fill] = PRIOR_DILATION
        return ev_out

    # ── step 8: LiDAR ground fill ─────────────────────────────────────────────

    @staticmethod
    def _lidar_ground_fill(ev, gnd_cnt):
        """Boost uncertain cells that LiDAR sees as flat ground."""
        uncertain = (ev >= 0.38) & (ev <= 0.62)
        confirmed = (gnd_cnt >= LIDAR_GND_MIN_HITS)
        boost = uncertain & confirmed
        if not boost.any():
            return ev
        ev_out = ev.copy()
        ev_out[boost] = np.maximum(ev_out[boost], LIDAR_GND_BOOST)
        return ev_out

    # ── step 9: connected road fill ───────────────────────────────────────────

    @staticmethod
    def _connected_road_fill(ev):
        """Flood fill from the vehicle footprint through all non-obstacle cells.

        Any cell reachable from the vehicle without crossing an obstacle is
        topologically connected to the road the vehicle is on — mark it drivable.
        This fills intersection branches and any blind spot that shares road
        connectivity with the vehicle, regardless of geometric distance.

        scipy.ndimage.label runs O(n) on the 300x300 grid (~90k cells) in <1ms."""
        traversable = ev > CC_TRAVERSABLE_MIN
        labeled, _ = sp_label(traversable)
        veh_lbl = labeled[_VR, _VC]
        if veh_lbl == 0:
            return ev
        connected = labeled == veh_lbl
        uncertain = (ev >= CC_UNCERTAIN_LO) & (ev <= CC_UNCERTAIN_HI)
        fill = connected & uncertain
        if not fill.any():
            return ev
        ev_out = ev.copy()
        ev_out[fill] = np.maximum(ev_out[fill], CC_FILL_VALUE)
        return ev_out

    # ── step 10: road corridor fill ───────────────────────────────────────────

    @staticmethod
    def _corridor_fill(ev):
        """Fill uncertain cells within the road corridor column-by-column.

        For each grid column (x = forward direction), span = first..last road
        row.  Any uncertain cell inside that span gets boosted to drivable.
        Cross-streets run in the row direction and have only 1-2 road cells
        per column, so they are naturally skipped by the min_cells_per_col=4
        guard — no road_cells threshold needed, intersections safe by geometry."""
        road = ev > 0.62
        if not road.any():
            return ev

        # Vectorised per-column span detection
        road_count  = road.sum(axis=0)                            # (300,)
        has_enough  = road_count >= 4                             # need >= 4 cells in col

        # first True row per column (argmax returns 0 when col is all-False — mask later)
        first_row = np.argmax(road, axis=0).astype(np.int32)
        last_row  = (GRID_SIZE - 1 - np.argmax(road[::-1, :], axis=0)).astype(np.int32)
        span      = last_row - first_row                          # corridor height

        # Guard: skip columns where span > 80 cells (16 m) — not a real road
        valid_cols = has_enough & (span <= 80)

        # Build corridor mask (300×300)
        rows       = np.arange(GRID_SIZE, dtype=np.int32).reshape(-1, 1)  # (300,1)
        in_corridor = (rows >= first_row) & (rows <= last_row) & valid_cols  # (300,300)

        fill = in_corridor & (ev > 0.28) & (ev < 0.70)
        ev_out = ev.copy()
        ev_out[fill] = np.maximum(ev_out[fill], 0.75)
        return ev_out

    # ── step 11: morpho cleanup ───────────────────────────────────────────────

    @staticmethod
    def _morpho_cleanup(ev):
        binary  = (ev > 0.55).astype(np.uint8) * 255
        k_open  = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        k_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  k_open)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, k_close)
        out = ev.copy()
        out[(binary > 0) & (cleaned == 0)] = np.minimum(out[(binary > 0) & (cleaned == 0)], 0.45)
        out[(binary == 0) & (cleaned > 0)] = np.maximum(out[(binary == 0) & (cleaned > 0)], 0.60)
        return out

    # ── publish loop ──────────────────────────────────────────────────────────

    def _publish_loop(self):
        with self._lock:
            sem_snap   = dict(self._sem)
            cloud_snap = self._lidar_cloud

        obs_sum, obs_count = self._camera_obs_grid(sem_snap)

        if cloud_snap is not None:
            gnd_cnt, obs_cnt = self._lidar_evidence_grid(cloud_snap)
        else:
            gnd_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)
            obs_cnt = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.int16)

        frame_obs, observed = self._fuse_camera_lidar(obs_sum, obs_count, gnd_cnt, obs_cnt)

        with self._lock:
            self._blend_evidence(frame_obs, observed)
            self._apply_priors()
            ev = self._evidence.copy()

        ev = self._smooth_boundaries(ev)           # step 6
        ev = self._dilate_into_blindspots(ev)      # step 7
        ev = self._lidar_ground_fill(ev, gnd_cnt)  # step 8
        ev = self._connected_road_fill(ev)         # step 9 — topology-aware fill
        ev = self._corridor_fill(ev)               # step 10 — column-wise corridor fill
        ev = self._morpho_cleanup(ev)              # step 11

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
