#!/usr/bin/env python3
"""
bev_geometry.py — shared camera-to-BEV projection geometry.

Extracted from perception_drivable_grid_node.py so every grid-publishing node
(drivable grid, lane grid, ...) stays pixel-aligned on the same 300x300
@0.2m/cell base_link grid and the same hardcoded camera extrinsics, instead of
each node re-deriving its own projection.

Cameras (hardcoded from carla_objects.json — camera TF frames do not exist):
  K: fx=fy=571.12, cx=400, cy=300  (fov=70, 800x600)
  R_cb(theta) = [[sin t, 0, cos t], [-cos t, 0, sin t], [0, -1, 0]]
  front: t=(0.7,-0.15,1.88) yaw=0    /semantic/front
  right: t=(0.7,-0.15,1.88) yaw=-70  /semantic/right
  left:  t=(0.7, 0.15,1.88) yaw=+70  /semantic/left
  back:  t=(-1.5,0.0, 1.88) yaw=180  /semantic/back (needs rgb_back in carla_objects.json)
"""

import math

import numpy as np

GRID_SIZE     = 300
RESOLUTION    = 0.2
ORIGIN_X      = -20.0
ORIGIN_Y      = -30.0
MAX_PROJ_DIST = 40.0

VEHICLE_COL = int((0.0 - ORIGIN_X) / RESOLUTION)   # 100
VEHICLE_ROW = int((0.0 - ORIGIN_Y) / RESOLUTION)   # 150

FX = FY = 571.12
CX, CY = 400.0, 300.0
IMG_W, IMG_H = 800, 600

K = np.array([[FX, 0.0, CX], [0.0, FY, CY], [0.0, 0.0, 1.0]], dtype=np.float64)
K_INV = np.linalg.inv(K)


def R_cb(yaw_deg):
    t = math.radians(yaw_deg)
    s, c = math.sin(t), math.cos(t)
    return np.array([[s, 0, c], [-c, 0, s], [0, -1, 0]], dtype=np.float64)


CAMERAS = [
    ('front', '/semantic/front',  np.array([ 0.70, -0.15, 1.88]), R_cb(  0.0)),
    ('right', '/semantic/right',  np.array([ 0.70, -0.15, 1.88]), R_cb(-70.0)),
    ('left',  '/semantic/left',   np.array([ 0.70,  0.15, 1.88]), R_cb( 70.0)),
    ('back',  '/semantic/back',   np.array([-1.50,  0.00, 1.88]), R_cb(180.0)),
]


class CamLUT:
    """Precomputed per-pixel -> BEV grid cell mapping for one camera."""

    __slots__ = ('gc', 'gr', 'valid', 'img_h', 'img_w')

    def __init__(self, t_base, R_cb_mat, img_h=IMG_H, img_w=IMG_W):
        us, vs = np.arange(img_w, dtype=np.float64), np.arange(img_h, dtype=np.float64)
        uu, vv = np.meshgrid(us, vs)
        uvh = np.stack([uu.ravel(), vv.ravel(), np.ones(img_h * img_w)])
        rays_cam = (K_INV @ uvh).T
        ray_base = (R_cb_mat @ rays_cam.T).T.astype(np.float32)

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
