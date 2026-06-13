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
