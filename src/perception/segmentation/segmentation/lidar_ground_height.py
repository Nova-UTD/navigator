#!/usr/bin/env python3
"""
lidar_ground_height.py — per-cell LiDAR ground height, for curb detection.

Author: Siddarth Nandyala
Email: siddarth.nandyala@utdallas.edu

A camera-based road classifier can't reliably tell a sidewalk from a road
surface -- both are flat and grayish. The physical curb between them is a
real height step LiDAR can see regardless. This module turns raw LiDAR
points into a per-cell ground-height grid that road_corridor.py uses to
refuse to cross a height jump, even where the camera/classifier says both
sides are "drivable".

Pure numpy, no rclpy — unit-testable standalone, same style as the other
segmentation modules.
"""

import numpy as np

from segmentation.bev_geometry import GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y

# Ground band for points considered part of the road/sidewalk surface itself
# (not overhead obstacles) -- same convention as lane_segmentation's old
# GROUND_Z_MIN/MAX and perception_drivable_grid_node's ground floor.
GROUND_Z_MIN = -0.30
GROUND_Z_MAX = 0.35

MIN_HITS = 2  # need >= this many ground points in a cell to trust its height


def parse_xyzi(msg):
    """Extract an Nx4 (x, y, z, intensity) array from a PointCloud2, reading
    field offsets dynamically rather than assuming a fixed struct layout.
    Returns None if the cloud has no intensity field."""
    n = msg.width * msg.height
    if n == 0:
        return None
    offsets = {}
    for f in msg.fields:
        if f.name in ('x', 'y', 'z', 'intensity'):
            offsets[f.name] = f.offset
    if not {'x', 'y', 'z', 'intensity'} <= offsets.keys():
        return None
    ps = msg.point_step
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, ps)
    cols = []
    for name in ('x', 'y', 'z', 'intensity'):
        off = offsets[name]
        cols.append(np.frombuffer(raw[:, off:off + 4].tobytes(), dtype=np.float32))
    points = np.stack(cols, axis=1)
    ok = np.isfinite(points).all(axis=1)
    return points[ok]


def ground_height_grid(points_xyzi, grid_size=GRID_SIZE, min_hits=MIN_HITS):
    """points_xyzi: Nx4 array of (x, y, z, intensity) in base_link.

    Returns a (grid_size, grid_size) float32 grid: mean ground-band z per
    cell, NaN where fewer than min_hits ground points were seen there (no
    LiDAR evidence either way -- not the same as "flat", so road_corridor
    treats NaN cells as never blocking).
    """
    height = np.full((grid_size, grid_size), np.nan, dtype=np.float32)
    if points_xyzi is None or len(points_xyzi) == 0:
        return height

    x, y, z = points_xyzi[:, 0], points_xyzi[:, 1], points_xyzi[:, 2]
    ground = (z >= GROUND_Z_MIN) & (z < GROUND_Z_MAX) & np.isfinite(x) & np.isfinite(y)
    x, y, z = x[ground], y[ground], z[ground]
    if len(z) == 0:
        return height

    gc = ((x - ORIGIN_X) / RESOLUTION).astype(np.int32)  # column = longitudinal (x)
    gr = ((y - ORIGIN_Y) / RESOLUTION).astype(np.int32)  # row = lateral (y)
    inb = (gc >= 0) & (gc < grid_size) & (gr >= 0) & (gr < grid_size)
    gc, gr, z = gc[inb], gr[inb], z[inb]
    if len(z) == 0:
        return height

    count = np.zeros((grid_size, grid_size), dtype=np.int32)
    total = np.zeros((grid_size, grid_size), dtype=np.float64)
    np.add.at(count, (gr, gc), 1)
    np.add.at(total, (gr, gc), z)

    trusted = count >= min_hits
    with np.errstate(divide='ignore', invalid='ignore'):
        mean = np.where(trusted, total / np.maximum(count, 1), np.nan)
    height[:] = mean.astype(np.float32)
    return height
