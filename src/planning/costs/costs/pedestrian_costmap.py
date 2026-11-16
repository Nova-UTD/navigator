"""
Package: costs
   File: pedestrian_costmap.py
 Author: Shrey Joshi

Pure-logic for the pedestrian-intent cost layer — cost mapping, base_link ->
grid projection, and disk painting. No ROS imports; deterministic and
unit-testable. The thin ROS node pedestrian_costmap_node.py delegates all math
here (mirrors how navigator_lane_change splits logic from the node).

Grid contract (must match grid_summation_node / the other cost layers):
  frame_id   = base_link
  size       = 151 x 151 cells, resolution 0.4 m/cell
  origin     = (x=-20.0 longitudinal, y=-30.0 lateral) in base_link
  ego cell   = (row=75, col=50)
  rows       = lateral axis    (+y = left,  -y = right)
  cols       = longitudinal    (+x = ahead, -x = behind)
"""

from typing import Optional, Tuple

import numpy as np


def distance_to_cost(distance_m: float, d_max_m: float) -> int:
    """Map a gap-to-road-edge distance to a cost magnitude in [0, 100].

    distance 0 (pedestrian in the road) -> 100; distance >= d_max -> 0; linear
    in between. This sets cost MAGNITUDE only — where the cost is painted is
    decided separately by pose_to_grid_coords.

    @param distance_m  Gap from pedestrian to road edge (m).
    @param d_max_m     Distance at which a pedestrian stops contributing (m).
    @return            Integer cost in [0, 100].
    """
    if d_max_m <= 0:
        return 0
    cost = 100.0 * (1.0 - distance_m / d_max_m)
    return int(round(float(np.clip(cost, 0, 100))))


def pose_to_grid_coords(pos_x_m: float, pos_y_m: float,
                        origin_x_m: float, origin_y_m: float,
                        resolution_m: float, grid_size: int
                        ) -> Optional[Tuple[int, int]]:
    """Convert a base_link metric pose to (row, col) grid indices.

    cols track longitudinal +x (ahead), rows track lateral +y (left), matching
    the layer's grid contract. Returns None if the pose falls outside the grid.

    @param pos_x_m       Forward position in base_link (m).
    @param pos_y_m       Lateral position in base_link (m).
    @param origin_x_m    Grid origin x (longitudinal min, m).
    @param origin_y_m    Grid origin y (lateral min, m).
    @param resolution_m  Cell size (m/cell).
    @param grid_size     Grid edge length (cells).
    @return              (row, col) tuple, or None if off-grid.
    """
    col = int(round((pos_x_m - origin_x_m) / resolution_m))
    row = int(round((pos_y_m - origin_y_m) / resolution_m))
    if 0 <= row < grid_size and 0 <= col < grid_size:
        return (row, col)
    return None


def paint_disk(grid: np.ndarray, row: int, col: int,
               radius_m: float, resolution_m: float, cost: int) -> None:
    """Paint a filled disk of `cost` into `grid`, combined via np.maximum.

    Mutates `grid` in place. Overlapping disks (multiple pedestrians) combine
    by taking the per-cell maximum, never summing. Out-of-bounds cells are
    skipped. Grid dtype is preserved (work in int16).

    @param grid          2D int16 cost array (modified in place).
    @param row           Disk center row index.
    @param col           Disk center col index.
    @param radius_m      Disk radius in meters (footprint + safety pad).
    @param resolution_m  Cell size (m/cell).
    @param cost          Cost value to paint.
    """
    radius_cells = radius_m / resolution_m
    r_int = int(np.ceil(radius_cells))
    rows, cols = grid.shape

    r_lo, r_hi = max(0, row - r_int), min(rows, row + r_int + 1)
    c_lo, c_hi = max(0, col - r_int), min(cols, col + r_int + 1)
    if r_lo >= r_hi or c_lo >= c_hi:
        return

    rr, cc = np.ogrid[r_lo:r_hi, c_lo:c_hi]
    mask = (rr - row) ** 2 + (cc - col) ** 2 <= radius_cells ** 2
    sub = grid[r_lo:r_hi, c_lo:c_hi]
    sub[mask] = np.maximum(sub[mask], cost)
