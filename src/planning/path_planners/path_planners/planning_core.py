"""
Costmap to planning problem pipeline shared by path_planner_node and the benchmark.

Package:   path_planners
Filename:  planning_core.py
Author:    Shifat Islam Santo

The costmap -> planning problem pipeline that path_planner_node runs before
handing a grid to a planner, with no ROS imports so the offline benchmark and
the unit tests exercise exactly the same steps as the live node:

1. reshape the OccupancyGrid data and rescale it to a 60 m x 60 m square
2. convert the start (vehicle) and goal (base_link metres) to grid cells
3. dilate obstacles by the vehicle half-width
4. snap a goal that landed inside an obstacle back toward the start

Grid convention (same as the node): row i is base_link y (left positive),
column j is base_link x (forward positive), cell (i, j) sits at
x = j * res - origin_x, y = i * res - origin_y.
"""

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import numpy as np
import scipy.ndimage

Cell = Tuple[int, int]

GRID_SPAN_M = 60.0  # the node always plans over a 60 m x 60 m window

# path_planner_node settings, shared so the benchmark cannot drift from the node
ORIGIN_X = 20.0  # vehicle distance from the back of the grid, metres
ORIGIN_Y = 30.0  # vehicle distance from the side of the grid, metres
OBSTACLE_THRESHOLD = 90  # cells at or above this are obstacles
OBSTACLE_PADDING = 5  # cells to pad around obstacles (5 = 1.0 m at 0.2 m, ~vehicle half-width)


@dataclass
class PlanningProblem:
    """A padded costmap plus the start and goal cells a planner receives."""

    costmap: np.ndarray  # padded, int32, shape (rows, cols)
    start: Cell
    goal: Cell
    goal_snapped: bool  # True if the requested goal cell was an obstacle


def costmap_to_array(data: Sequence[int], height: int, width: int,
                     resolution: float) -> np.ndarray:
    """Reshape OccupancyGrid data and rescale it to GRID_SPAN_M square."""
    target_cells = int(GRID_SPAN_M / resolution)
    costmap_np = np.asarray(data, dtype=np.int32).reshape(height, width)
    if height != target_cells or width != target_cells:
        # Nearest neighbour so obstacles remain discrete
        zoom = (target_cells / height, target_cells / width)
        costmap_np = scipy.ndimage.zoom(costmap_np, zoom, order=0).astype(np.int32)
    return costmap_np


def pad_obstacles(cmap: np.ndarray, threshold: int, padding: int) -> np.ndarray:
    """Dilate every cell at or above threshold by `padding` cells and mark it 100."""
    obstacles = 0 * cmap
    obstacles[np.where(cmap >= threshold)] = 1
    struct2 = scipy.ndimage.generate_binary_structure(2, 2)  # 3x3 grid all true
    obstacles = scipy.ndimage.binary_dilation(
        obstacles, structure=struct2, iterations=padding).astype(cmap.dtype)

    padded_costmap = cmap.copy()
    padded_costmap[obstacles > 0] = 100
    return padded_costmap


def xy_to_cell(x: float, y: float, resolution: float, origin_x: float,
               origin_y: float, shape: Tuple[int, int]) -> Cell:
    """Convert a base_link point to a grid cell, clamped to the grid."""
    i = int(round((y + origin_y) / resolution))
    j = int(round((x + origin_x) / resolution))
    return (max(0, min(i, shape[0] - 1)), max(0, min(j, shape[1] - 1)))


def cells_to_xy(path: Sequence[Tuple[float, float]], resolution: float,
                origin_x: float, origin_y: float) -> List[Tuple[float, float]]:
    """Convert (row, col) path points back to base_link (x, y) metres."""
    return [(p[1] * resolution - origin_x, p[0] * resolution - origin_y) for p in path]


def snap_goal(costmap: np.ndarray, start: Cell, goal: Cell,
              threshold: int) -> Optional[Cell]:
    """
    Return the goal, or the nearest free cell on the line back to the start.

    Returns None if that whole line is blocked. A blocked goal equal to the
    start is returned unchanged, as the node did.
    """
    if costmap[goal] < threshold:
        return goal
    height, width = costmap.shape
    steps = max(abs(goal[0] - start[0]), abs(goal[1] - start[1]))
    if steps == 0:
        return goal
    for s in range(steps, -1, -1):
        t = s / steps
        ci = int(round(start[0] + t * (goal[0] - start[0])))
        cj = int(round(start[1] + t * (goal[1] - start[1])))
        ci = max(0, min(ci, height - 1))
        cj = max(0, min(cj, width - 1))
        if costmap[ci, cj] < threshold:
            return (ci, cj)
    return None


def build_problem(costmap_np: np.ndarray, resolution: float, goal_x: float,
                  goal_y: float, origin_x: float, origin_y: float,
                  threshold: int, padding: int) -> Optional[PlanningProblem]:
    """
    Run the node's full pre-planning pipeline on a rescaled costmap.

    Returns None when the goal is blocked and no free cell exists on the
    line back to the start (the node publishes nothing in that case).
    """
    shape = costmap_np.shape
    start = xy_to_cell(0.0, 0.0, resolution, origin_x, origin_y, shape)
    goal = xy_to_cell(goal_x, goal_y, resolution, origin_x, origin_y, shape)
    padded = pad_obstacles(costmap_np, threshold, padding)
    snapped = snap_goal(padded, start, goal, threshold)
    if snapped is None:
        return None
    return PlanningProblem(padded, start, snapped, snapped != goal)


def rolling_smoothing(path, look_ahead=2, depth=3):
    """
    Smooth a path by moving each point toward a future point.

    Parameters
    ----------
    path : list of (row, col) tuples
        The path to smooth.
    look_ahead : int
        How many points ahead to look.
    depth : int
        Number of smoothing iterations.

    Returns
    -------
    The smoothed path as a list of (row, col) tuples

    """
    if len(path) < look_ahead:
        return path

    t = 1.0 / look_ahead
    for d in range(depth):
        new_path = [path[0]]
        for i in range(1, len(path) - look_ahead):
            x0 = path[i - 1][0]
            y0 = path[i - 1][1]

            dx = path[i - 1 + look_ahead][0] - x0
            dy = path[i - 1 + look_ahead][1] - y0
            new_path.append((x0 + t * dx, y0 + t * dy))
        new_path.extend(path[-look_ahead:])
        path = new_path

    return path
