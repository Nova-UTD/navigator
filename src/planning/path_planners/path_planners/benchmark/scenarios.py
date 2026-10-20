"""
Costmaps the benchmark plans over.

Package:   path_planners
Filename:  benchmark/scenarios.py
Author:    Shifat Islam Santo

Synthetic scenarios are drawn in base_link metres on the same grid the stack
publishes on /grid/steering_cost (param/global_parameters.yaml: 60 m x 60 m at
0.2 m, vehicle 20 m from the back and 30 m from the left) with the values the
cost layers produce: 50 off-route, 10 in the route corridor (+/-2 m), 0 on the
route centreline (route_costmap_node.py) and 100 for occupied cells.

Recorded scenarios are .npz files holding a real /grid/steering_cost frame and
the /planning/path_goal that was live at that moment (see extract_costmaps.py).
"""

from dataclasses import dataclass, field
import glob
import os
from typing import Callable, Dict, List, Optional, Tuple

import numpy as np
from path_planners.planning_core import GRID_SPAN_M as SPAN_M, ORIGIN_X, ORIGIN_Y
import scipy.ndimage

RESOLUTION = 0.2  # param/global_parameters.yaml occupancy_grids.resolution

OFF_ROUTE = 50
CORRIDOR = 10
CENTRELINE = 0
OCCUPIED = 100
CORRIDOR_HALF_WIDTH_CELLS = 10  # route_costmap_node HALF_W


@dataclass
class Scenario:
    """One costmap frame plus the goal the node would plan to."""

    name: str
    costmap: np.ndarray  # raw grid as published, before rescale and padding
    resolution: float
    goal_xy: Tuple[float, float]  # base_link metres
    description: str
    expect_reachable: Optional[bool] = None  # None = let the reference decide
    tags: List[str] = field(default_factory=list)


class Canvas:
    """A blank steering-cost grid addressed in base_link metres."""

    def __init__(self, resolution=RESOLUTION, span_m=SPAN_M):
        self.res = resolution
        n = int(round(span_m / resolution))
        rows, cols = np.indices((n, n))
        self.x = cols * resolution - ORIGIN_X
        self.y = rows * resolution - ORIGIN_Y
        self.grid = np.full((n, n), OFF_ROUTE, dtype=np.int32)

    def route(self, centreline_xy: np.ndarray):
        """Paint the route corridor and centreline the route cost layer draws."""
        n = self.grid.shape[0]
        line = np.zeros_like(self.grid, dtype=bool)
        i = np.round((centreline_xy[:, 1] + ORIGIN_Y) / self.res).astype(int)
        j = np.round((centreline_xy[:, 0] + ORIGIN_X) / self.res).astype(int)
        keep = (i >= 0) & (i < n) & (j >= 0) & (j < n)
        line[i[keep], j[keep]] = True
        # Chebyshev distance matches the square stamp route_costmap_node uses
        dist = scipy.ndimage.distance_transform_cdt(~line, metric='chessboard')
        self.grid = np.where(dist <= CORRIDOR_HALF_WIDTH_CELLS,
                             np.minimum(self.grid, CORRIDOR), self.grid)
        self.grid[line] = CENTRELINE
        return self

    def box(self, cx, cy, length, width, yaw_deg=0.0):
        """Occupy an oriented rectangle, e.g. a parked car."""
        yaw = np.radians(yaw_deg)
        dx, dy = self.x - cx, self.y - cy
        along = dx * np.cos(yaw) + dy * np.sin(yaw)
        across = -dx * np.sin(yaw) + dy * np.cos(yaw)
        self.grid[(np.abs(along) <= length / 2) & (np.abs(across) <= width / 2)] = OCCUPIED
        return self

    def disc(self, cx, cy, radius):
        """Occupy a circle, e.g. a pedestrian or a cone."""
        self.grid[(self.x - cx) ** 2 + (self.y - cy) ** 2 <= radius ** 2] = OCCUPIED
        return self

    def wall_x(self, x, y_from, y_to, thickness=0.4):
        """Occupy a wall across the road at longitudinal position x."""
        self.grid[(np.abs(self.x - x) <= thickness / 2)
                  & (self.y >= y_from) & (self.y <= y_to)] = OCCUPIED
        return self

    def wall_y(self, y, x_from, x_to, thickness=0.4):
        """Occupy a wall along the road at lateral position y."""
        self.grid[(np.abs(self.y - y) <= thickness / 2)
                  & (self.x >= x_from) & (self.x <= x_to)] = OCCUPIED
        return self

    def clear_disc(self, cx, cy, radius):
        """Remove anything occupying a circle (keeps start and goal free)."""
        hit = (self.x - cx) ** 2 + (self.y - cy) ** 2 <= radius ** 2
        self.grid[hit & (self.grid == OCCUPIED)] = OFF_ROUTE
        return self


def straight_route(length=40.0, y=0.0):
    xs = np.arange(-ORIGIN_X, length, RESOLUTION / 2)
    return np.stack([xs, np.full_like(xs, y)], axis=1)


def arc_route(radius=25.0, sweep_deg=70.0):
    """Return a left-hand bend that starts at the vehicle heading straight ahead."""
    back = straight_route(0.0)
    t = np.radians(np.linspace(0.0, sweep_deg, 400))
    arc = np.stack([radius * np.sin(t), radius * (1 - np.cos(t))], axis=1)
    return np.concatenate([back, arc]), arc[-1]


def _straight_clear():
    c = Canvas().route(straight_route())
    return Scenario('straight_clear', c.grid, RESOLUTION, (25.0, 0.0),
                    'Empty straight road, goal 25 m ahead on the centreline.', True,
                    ['baseline'])


def _parked_car():
    c = Canvas().route(straight_route()).box(12.0, 0.0, 4.5, 2.0)
    return Scenario('parked_car', c.grid, RESOLUTION, (25.0, 0.0),
                    'A car parked across the centreline at 12 m; the path must swerve.',
                    True, ['obstacle'])


def _slalom():
    c = Canvas().route(straight_route())
    for x, y in ((8.0, 1.0), (16.0, -1.0), (24.0, 1.0)):
        c.box(x, y, 4.5, 2.0)
    return Scenario('slalom', c.grid, RESOLUTION, (32.0, 0.0),
                    'Three staggered cars at 8, 16, 24 m alternating sides.', True,
                    ['obstacle'])


def _wall_with_gap():
    c = Canvas().route(straight_route())
    c.wall_x(15.0, -30.0, 4.0).wall_x(15.0, 7.0, 30.0)
    return Scenario('wall_with_gap', c.grid, RESOLUTION, (25.0, 0.0),
                    'Wall across the road at 15 m with one 3 m gap 4-7 m to the left '
                    '(1 m wide after padding).', True, ['narrow'])


def _u_trap():
    c = Canvas().route(straight_route())
    c.wall_x(14.0, -4.0, 4.0).wall_y(-4.0, 6.0, 14.0).wall_y(4.0, 6.0, 14.0)
    return Scenario('u_trap', c.grid, RESOLUTION, (25.0, 0.0),
                    'U-shaped obstacle open toward the car, goal behind it: a local '
                    'minimum for goal-greedy methods.', True, ['local-minimum'])


def _detour_beyond_margin():
    c = Canvas().route(straight_route())
    c.wall_x(5.0, -30.0, 11.0).wall_x(5.0, 14.0, 30.0)
    return Scenario('detour_beyond_margin', c.grid, RESOLUTION, (10.0, 0.0),
                    'Wall 5 m ahead, only gap 11-14 m to the left; the detour leaves '
                    'a 40-cell box around start and goal.', True, ['completeness'])


def _goal_in_obstacle():
    c = Canvas().route(straight_route()).box(20.0, 0.0, 4.5, 2.0)
    return Scenario('goal_in_obstacle', c.grid, RESOLUTION, (20.0, 0.0),
                    'Goal lands inside a parked car; the node snaps it back toward '
                    'the start.', True, ['goal-snap'])


def _unreachable():
    c = Canvas().route(straight_route()).wall_x(10.0, -30.0, 30.0)
    return Scenario('unreachable', c.grid, RESOLUTION, (25.0, 0.0),
                    'Wall across the whole grid at 10 m. The right answer is no path.',
                    False, ['no-path'])


def _curve_left():
    centreline, end = arc_route()
    c = Canvas().route(centreline)
    return Scenario('curve_left', c.grid, RESOLUTION, (float(end[0]), float(end[1])),
                    'Left-hand bend of radius 25 m; goal at the end of a 70 degree arc.',
                    True, ['baseline'])


def _lane_closure():
    c = Canvas().route(straight_route())
    for k in range(7):
        c.disc(10.0 + 1.5 * k, -1.8 + 0.6 * k, 0.3)
    return Scenario('lane_closure', c.grid, RESOLUTION, (30.0, 0.0),
                    'Diagonal line of cones closing the route from 10 to 19 m.', True,
                    ['obstacle'])


def _clutter(seed):
    def build():
        rng = np.random.default_rng(seed)
        c = Canvas().route(straight_route())
        for _ in range(28):
            c.disc(rng.uniform(3.0, 30.0), rng.uniform(-9.0, 9.0), rng.uniform(0.4, 1.0))
        c.clear_disc(0.0, 0.0, 2.5).clear_disc(32.0, 0.0, 2.5)
        return Scenario(f'clutter_s{seed}', c.grid, RESOLUTION, (32.0, 0.0),
                        f'28 random discs (seed {seed}) on and beside the road.', None,
                        ['clutter'])
    return build


SYNTHETIC: Dict[str, Callable[[], Scenario]] = {
    'straight_clear': _straight_clear,
    'curve_left': _curve_left,
    'parked_car': _parked_car,
    'slalom': _slalom,
    'lane_closure': _lane_closure,
    'wall_with_gap': _wall_with_gap,
    'u_trap': _u_trap,
    'detour_beyond_margin': _detour_beyond_margin,
    'goal_in_obstacle': _goal_in_obstacle,
    'unreachable': _unreachable,
}
for _seed in range(3):
    SYNTHETIC[f'clutter_s{_seed}'] = _clutter(_seed)


def load_recorded(path: str) -> Scenario:
    """Load one recorded frame written by extract_costmaps.py."""
    with np.load(path) as f:
        name = str(f['name']) if 'name' in f else os.path.splitext(os.path.basename(path))[0]
        return Scenario(name, f['costmap'].astype(np.int32), float(f['resolution']),
                        (float(f['goal_x']), float(f['goal_y'])),
                        f'Recorded /grid/steering_cost frame from {os.path.basename(path)}.',
                        None, ['recorded'])


def collect(synthetic: Optional[List[str]], recorded: List[str]) -> List[Scenario]:
    """Build the requested synthetic scenarios and load every recorded .npz."""
    names = list(SYNTHETIC) if synthetic is None else synthetic
    unknown = [n for n in names if n not in SYNTHETIC]
    if unknown:
        raise ValueError(f"Unknown scenarios {unknown}. Known: {', '.join(SYNTHETIC)}")
    scenarios = [SYNTHETIC[n]() for n in names]
    for item in recorded:
        files = sorted(glob.glob(os.path.join(item, '*.npz'))) if os.path.isdir(item) else [item]
        scenarios.extend(load_recorded(f) for f in files)
    return scenarios
