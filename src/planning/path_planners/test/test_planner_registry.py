"""Tests for planner_registry: selection by name and the uniform plan() call."""

import importlib.util
import subprocess
import sys

import numpy as np
from path_planners.planner_registry import (
    DEFAULT_PLANNER, make_planner, PLANNER_NAMES, UnknownPlannerError)
import pytest

NEEDS = {'ara-star': 'networkx', 'neural-network': 'torch'}


def _available(name):
    module = NEEDS.get(name)
    return module is None or importlib.util.find_spec(module) is not None


def test_default_is_dijkstra_and_listed():
    assert DEFAULT_PLANNER == 'dijkstra'
    assert DEFAULT_PLANNER in PLANNER_NAMES
    assert len(set(PLANNER_NAMES)) == len(PLANNER_NAMES) == 5


def test_unknown_name_lists_every_valid_planner():
    with pytest.raises(UnknownPlannerError) as err:
        make_planner('dijkstras')
    message = str(err.value)
    assert "'dijkstras'" in message
    for name in PLANNER_NAMES:
        assert name in message


def test_dijkstra_does_not_import_heavy_dependencies():
    code = ('import sys; from path_planners.planner_registry import make_planner; '
            "make_planner('dijkstra'); "
            "print(','.join(m for m in ('torch', 'networkx') if m in sys.modules))")
    out = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True,
                         check=True)
    assert out.stdout.strip() == ''


@pytest.mark.parametrize('name', PLANNER_NAMES)
def test_every_planner_runs_through_the_uniform_call(name):
    if not _available(name):
        pytest.skip(f'{NEEDS[name]} not installed')
    costmap = np.zeros((40, 40), dtype=np.int32)
    costmap[10:30, 20] = 100  # a wall to go around
    planner = make_planner(name)
    assert planner.name == name
    path = planner(costmap, (20, 5), (20, 35), 90)
    # Wiring only: each planner returns None or a sequence of (row, col) points
    assert path is None or all(len(p) == 2 for p in path)


def test_dijkstra_plans_around_a_wall():
    costmap = np.zeros((40, 40), dtype=np.int32)
    costmap[5:35, 20] = 100
    path = make_planner('dijkstra')(costmap, (20, 5), (20, 35), 90)
    assert path[0] == (20, 5) and path[-1] == (20, 35)
    assert all(costmap[p] < 90 for p in path)
