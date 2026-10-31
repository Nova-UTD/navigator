"""
How a planner path is scored against an exact reference.

Package:   path_planners
Filename:  benchmark/metrics.py
Author:    Shifat Islam Santo

Every number here is computed by this module,
never by the planner under test, so all five planners are held to one ruler.

- reference: an exact 8-connected shortest path over the whole padded grid
  (scipy csgraph Dijkstra), with the same edge cost the stack's Dijkstra uses,
  (cell value + 1) x step length. It decides whether a path exists at all.
- validity: the path starts at the start, ends at the goal, and every point
  sampled along it keeps the path centre at least the vehicle half-width from
  every real (unpadded) obstacle. The node encodes the half-width as obstacle
  padding (5 cells = 1.0 m), so that is the required clearance, less half a
  cell of grid rounding. Checked on the raw path and again after the node's
  rolling_smoothing, because the smoothed path is what gets published.
  Entering a padded cell is reported separately and does not fail a path: the
  padding is a 5-cell square, so its diagonal corners reach ~1.4 m out and a
  path can clip them while keeping more than 1.0 m of real clearance.
- cost: the raw path rasterized to 8-connected cells and charged the graph's
  own edge cost, divided by the reference cost (1.0 = as cheap as exact; a
  path can never score below 1.0 under this rule).
- length, clearance to the nearest unpadded obstacle, and turning, in metres
  and degrees.
"""

import math
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from path_planners.planning_core import rolling_smoothing
import scipy.ndimage
import scipy.sparse
import scipy.sparse.csgraph

SAMPLE_STEP = 0.25  # cells between collision/cost samples along a path
ENDPOINT_TOLERANCE = 1.5  # cells; smoothing and float paths land near, not on
DIRS = ((-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1))


def reference_path(costmap: np.ndarray, start, goal,
                   threshold: int) -> Optional[List[Tuple[int, int]]]:
    """Exact minimum-cost 8-connected path over the full grid, or None."""
    rows, cols = costmap.shape
    free = costmap < threshold
    if not free[start] or not free[goal]:
        return None
    index = np.arange(rows * cols).reshape(rows, cols)
    src, dst, weight = [], [], []
    for dr, dc in DIRS:
        r0, r1 = max(0, -dr), rows - max(0, dr)
        c0, c1 = max(0, -dc), cols - max(0, dc)
        a = free[r0:r1, c0:c1]
        b = free[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        both = a & b
        step = math.sqrt(2.0) if dr and dc else 1.0
        src.append(index[r0:r1, c0:c1][both])
        dst.append(index[r0 + dr:r1 + dr, c0 + dc:c1 + dc][both])
        weight.append((costmap[r0 + dr:r1 + dr, c0 + dc:c1 + dc][both] + 1.0) * step)
    graph = scipy.sparse.csr_matrix(
        (np.concatenate(weight), (np.concatenate(src), np.concatenate(dst))),
        shape=(rows * cols, rows * cols))
    s, g = int(index[start]), int(index[goal])
    dist, pred = scipy.sparse.csgraph.dijkstra(
        graph, directed=True, indices=s, return_predecessors=True)
    if not np.isfinite(dist[g]):
        return None
    path, node = [], g
    while node != s:
        path.append(divmod(int(node), cols))
        node = pred[node]
    path.append(start)
    path.reverse()
    return path


def as_points(path: Sequence) -> np.ndarray:
    """Planner output (tuples, numpy ints or floats) as an (N, 2) float array."""
    return np.asarray([(float(p[0]), float(p[1])) for p in path], dtype=float)


def densify(points: np.ndarray, step: float = SAMPLE_STEP) -> np.ndarray:
    """Sample a polyline about every `step` cells, keeping every vertex."""
    if len(points) < 2:
        return points
    out = [points[:1]]
    for a, b in zip(points[:-1], points[1:]):
        # An odd sample count keeps samples off a diagonal step's midpoint, the
        # cell corner it only touches, which would round onto a third cell
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / step))) | 1
        t = np.arange(1, n + 1)[:, None] / n
        out.append(a + t * (b - a))
    return np.concatenate(out)


def first_collision(samples: np.ndarray, costmap: np.ndarray,
                    threshold: int) -> Optional[int]:
    """Index of the first sample on an obstacle or off the grid, else None."""
    cells = np.round(samples).astype(int)
    rows, cols = costmap.shape
    inside = ((cells[:, 0] >= 0) & (cells[:, 0] < rows)
              & (cells[:, 1] >= 0) & (cells[:, 1] < cols))
    blocked = ~inside
    blocked[inside] = costmap[cells[inside, 0], cells[inside, 1]] >= threshold
    hits = np.flatnonzero(blocked)
    return int(hits[0]) if len(hits) else None


def rasterize(points: np.ndarray) -> np.ndarray:
    """Return the 8-connected cells a polyline passes through (Bresenham per segment)."""
    verts = np.round(points).astype(int)
    cells = [tuple(verts[0])]
    for (r0, c0), (r1, c1) in zip(verts[:-1], verts[1:]):
        dr, dc = abs(r1 - r0), abs(c1 - c0)
        sr, sc = (1 if r1 > r0 else -1), (1 if c1 > c0 else -1)
        err, r, c = dr - dc, r0, c0
        while (r, c) != (r1, c1):
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc
            cells.append((r, c))
    return np.asarray(cells)


def path_cost(points: np.ndarray, costmap: np.ndarray) -> float:
    """
    Return the path's cost under the planner graph's own edge rule.

    The path is rasterized to 8-connected cells and each step is charged
    (value of the cell entered + 1) x step length. For a grid path
    this is exactly what reference_path minimizes, so no path can score below
    the reference.
    """
    cells = rasterize(points)
    if len(cells) < 2:
        return 0.0
    cells = np.clip(cells, 0, np.array(costmap.shape) - 1)
    steps = np.abs(np.diff(cells, axis=0))
    keep = steps.any(axis=1)
    length = np.where(steps[:, 0] & steps[:, 1], math.sqrt(2.0), 1.0)[keep]
    entered = cells[1:][keep]
    return float(np.sum((costmap[entered[:, 0], entered[:, 1]] + 1.0) * length))


def length_cells(points: np.ndarray) -> float:
    if len(points) < 2:
        return 0.0
    return float(np.sum(np.linalg.norm(np.diff(points, axis=0), axis=1)))


def turning_deg(points: np.ndarray) -> Tuple[float, float]:
    """(largest single heading change, total heading change) along the path."""
    seg = np.diff(points, axis=0)
    seg = seg[np.linalg.norm(seg, axis=1) > 1e-9]
    if len(seg) < 2:
        return 0.0, 0.0
    heading = np.arctan2(seg[:, 0], seg[:, 1])
    turn = np.abs((np.diff(heading) + np.pi) % (2 * np.pi) - np.pi)
    return float(np.degrees(turn.max())), float(np.degrees(turn.sum()))


def obstacle_distance(raw_costmap: np.ndarray, threshold: int) -> Optional[np.ndarray]:
    """Distance in cells from every cell to the nearest unpadded obstacle."""
    obstacles = raw_costmap >= threshold
    if not obstacles.any():
        return None
    return scipy.ndimage.distance_transform_edt(~obstacles)


def required_clearance_m(padding_cells: int, resolution: float) -> float:
    """Vehicle half-width as the node encodes it, less half a cell of rounding."""
    return (padding_cells - 0.5) * resolution


def _clearance_profile(samples, clearance_map, resolution):
    """Clearance in metres at each sample (inf everywhere if there are no obstacles)."""
    if clearance_map is None:
        return np.full(len(samples), np.inf)
    cells = np.clip(np.round(samples).astype(int), 0, np.array(clearance_map.shape) - 1)
    return clearance_map[cells[:, 0], cells[:, 1]] * resolution


def evaluate(path, problem, threshold: int, resolution: float,
             clearance_map: Optional[np.ndarray], ref_cost: Optional[float],
             padding_cells: int) -> Dict:
    """Score one planner output against the problem it was given."""
    row = {'returned_path': bool(path is not None and len(path) > 0)}
    if not row['returned_path']:
        return row
    raw = as_points(path)
    smooth = as_points(rolling_smoothing([tuple(p) for p in raw]))
    start = np.asarray(problem.start, dtype=float)
    goal = np.asarray(problem.goal, dtype=float)
    need = required_clearance_m(padding_cells, resolution)

    raw_samples = densify(raw)
    smooth_samples = densify(smooth)
    raw_clear = _clearance_profile(raw_samples, clearance_map, resolution)
    smooth_clear = _clearance_profile(smooth_samples, clearance_map, resolution)
    too_close = np.flatnonzero(raw_clear < need)

    row.update({
        'points': len(raw),
        'starts_at_start': bool(np.linalg.norm(raw[0] - start) <= ENDPOINT_TOLERANCE),
        'ends_at_goal': bool(np.linalg.norm(raw[-1] - goal) <= ENDPOINT_TOLERANCE),
        'clear': bool(raw_clear.min() >= need),
        'clear_smoothed': bool(smooth_clear.min() >= need),
        'centre_hits_obstacle': bool(min(raw_clear.min(), smooth_clear.min()) == 0.0),
        'enters_padding': first_collision(raw_samples, problem.costmap, threshold) is not None,
        'enters_padding_smoothed':
            first_collision(smooth_samples, problem.costmap, threshold) is not None,
        'first_too_close_m': (None if len(too_close) == 0 else round(
            length_cells(raw_samples[:too_close[0] + 1]) * resolution, 2)),
        'min_clearance_m': (None if clearance_map is None else float(smooth_clear.min())),
        'length_m': length_cells(smooth) * resolution,
        'cost': path_cost(raw, problem.costmap),
    })
    row['valid'] = (row['starts_at_start'] and row['ends_at_goal']
                    and row['clear'] and row['clear_smoothed'])
    row['cost_ratio'] = (row['cost'] / ref_cost) if ref_cost else None
    row['max_turn_deg'], row['total_turn_deg'] = turning_deg(smooth)
    return row
