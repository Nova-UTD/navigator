"""
Path-relative (Frenet) projection utilities shared across behavior modules.

Coordinate convention:
  rel_s  > 0  → ahead of ego along the path
  rel_s  < 0  → behind ego along the path
  lat    > 0  → LEFT of path forward direction (right-hand rule)
  lat    < 0  → RIGHT of path forward direction
"""

import math
from typing import List, Tuple

PathPoints = List[Tuple[float, float, float]]


def _path_tangent_at(ego_x: float, ego_y: float, path: PathPoints) -> Tuple[float, float]:
    """
    Return the unit forward tangent vector of the path at the point closest to
    (ego_x, ego_y).  Falls back to the first segment if the path has only 2 points.
    """
    best_dist = float("inf")
    best_seg = 0

    prev = None
    for idx, pt in enumerate(path):
        if prev is None:
            prev = pt
            continue

        dx = pt[0] - prev[0]
        dy = pt[1] - prev[1]
        seg_len = math.sqrt(dx * dx + dy * dy)
        if seg_len < 1e-6:
            prev = pt
            continue

        ux, uy = dx / seg_len, dy / seg_len
        # scalar projection of ego onto segment
        t = (ego_x - prev[0]) * ux + (ego_y - prev[1]) * uy
        t = max(0.0, min(seg_len, t))
        proj_x = prev[0] + t * ux
        proj_y = prev[1] + t * uy
        dist = math.sqrt((ego_x - proj_x) ** 2 + (ego_y - proj_y) ** 2)

        if dist < best_dist:
            best_dist = dist
            best_seg = idx - 1  # segment index = prev index
            best_ux, best_uy = ux, uy

        prev = pt

    if best_dist == float("inf"):
        # Fall back to first segment
        if len(path) >= 2:
            dx = path[1][0] - path[0][0]
            dy = path[1][1] - path[0][1]
            seg_len = math.sqrt(dx * dx + dy * dy)
            if seg_len > 1e-6:
                return dx / seg_len, dy / seg_len
        return 1.0, 0.0

    return best_ux, best_uy


def project_relative_to_ego(
    ox: float,
    oy: float,
    path: PathPoints,
    ego_x: float,
    ego_y: float,
) -> Tuple[float, float]:
    """
    Return (rel_s, lateral) of point (ox, oy) in the Frenet frame centred on
    the ego's position along the path.

    Uses the path tangent at the closest path point to ego as the local axis.
    This is exact on straight segments and a good approximation on gentle curves.

    rel_s > 0  → ahead of ego
    lat   > 0  → left of ego heading (right-hand rule in 2D)
    """
    ux, uy = _path_tangent_at(ego_x, ego_y, path)

    rx = ox - ego_x
    ry = oy - ego_y

    rel_s = rx * ux + ry * uy          # longitudinal: dot product
    lat = ux * ry - uy * rx            # lateral: 2D cross product (z-component)

    return rel_s, lat
