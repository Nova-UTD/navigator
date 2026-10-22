"""
Every planner path_planner_node can run, selected by name.

Package:   path_planners
Filename:  planner_registry.py
Author:    Shifat Islam Santo

One place that knows every planner path_planner_node can run.

Each planner class has its own calling convention (ARA* takes its inputs as
attributes, DP runs value iteration then extracts, the rest take arguments).
make_planner() hides that behind one signature,

    plan(costmap, start, goal, obstacle_threshold) -> list of (row, col) or None

so the node, the launch files and the offline benchmark all select a planner
by name and call it the same way. Planner modules are imported lazily, so
choosing dijkstra does not pull in torch or networkx.
"""

import os
import sys
from typing import Callable, List, Optional, Tuple

import numpy as np

Cell = Tuple[int, int]
Path = Optional[List[Tuple[float, float]]]

DEFAULT_PLANNER = 'dijkstra'

# Order matters only for help text and the benchmark's column order.
PLANNER_NAMES = (
    'dijkstra',
    'ara-star',
    'dynamic-programming',
    'neural-network',
    'trrt-star',
)


class UnknownPlannerError(ValueError):
    """Raised when a planner name is not in PLANNER_NAMES."""

    def __init__(self, name):
        super().__init__(
            f"Unknown planner '{name}'. Valid planners: {', '.join(PLANNER_NAMES)}")
        self.name = name


class Planner:
    """A named planner instance with the uniform plan() call."""

    def __init__(self, name: str, impl, plan_fn: Callable[..., Path]):
        self.name = name
        self.impl = impl  # the underlying planner object, state and all
        self._plan_fn = plan_fn

    def __call__(self, costmap: np.ndarray, start: Cell, goal: Cell,
                 obstacle_threshold: int) -> Path:
        return self._plan_fn(costmap, start, goal, obstacle_threshold)


def _make_dijkstra(options):
    from path_planners.dijkstra_path_planner import DijkstraPathPlanner
    impl = DijkstraPathPlanner()

    def plan(costmap, start, goal, obstacle_threshold):
        return impl.shortest_path(costmap, start, goal, obstacle_threshold)
    return impl, plan


def _make_ara_star(options):
    from path_planners.arastar_path_planner import ARAStarPlanner
    impl = ARAStarPlanner()

    def plan(costmap, start, goal, obstacle_threshold):
        impl.s_start = start
        impl.s_goal = goal
        impl.cmap = costmap
        impl.obstacle_threshold = obstacle_threshold
        impl.create_graph_from_costmap()
        return impl.arastar()
    return impl, plan


def _make_dynamic_programming(options):
    from path_planners.dp_path_planner import DPPathPlanner
    impl = DPPathPlanner()

    def plan(costmap, start, goal, obstacle_threshold):
        impl.costmap_data = costmap
        impl.obstacle_threshold = obstacle_threshold
        if not impl.run_value_iteration(start[0], start[1], goal[0], goal[1]):
            return None
        return impl.extract_path(start, goal)
    return impl, plan


def _make_neural_network(options):
    # neural_network_path_planner imports its UNet as a top-level module
    package_dir = os.path.dirname(os.path.abspath(__file__))
    if package_dir not in sys.path:
        sys.path.append(package_dir)
    from path_planners.neural_network_path_planner import NeuralPathPlanner
    impl = NeuralPathPlanner(model_path=options.get('model_path'))

    def plan(costmap, start, goal, obstacle_threshold):
        # The network has no obstacle threshold input; it only sees the costmap
        return impl.predict_path(costmap, start, goal)
    return impl, plan


def _make_trrt_star(options):
    from path_planners.trrtstar_path_planner import TRRTStarPathPlanner
    impl = TRRTStarPathPlanner()

    def plan(costmap, start, goal, obstacle_threshold):
        return impl.trrtstar_path(costmap, start, goal, obstacle_threshold)
    return impl, plan


_FACTORIES = {
    'dijkstra': _make_dijkstra,
    'ara-star': _make_ara_star,
    'dynamic-programming': _make_dynamic_programming,
    'neural-network': _make_neural_network,
    'trrt-star': _make_trrt_star,
}
assert tuple(_FACTORIES) == PLANNER_NAMES


def make_planner(name: str, **options) -> Planner:
    """
    Build the planner called `name`.

    Options: model_path (neural-network only) is the UNet checkpoint to load;
    the node passes none, which keeps today's behaviour of an untrained UNet.

    Raises UnknownPlannerError for a name not in PLANNER_NAMES.
    """
    factory = _FACTORIES.get(name)
    if factory is None:
        raise UnknownPlannerError(name)
    impl, plan = factory(options)
    return Planner(name, impl, plan)
