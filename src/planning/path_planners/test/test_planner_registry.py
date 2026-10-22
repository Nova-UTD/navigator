"""Tests for planner_registry: selection by name and the uniform plan() call."""

import importlib.util
from pathlib import Path
import re
import subprocess
import sys

import numpy as np
from path_planners.planner_registry import (
    DEFAULT_PLANNER, make_planner, PLANNER_NAMES, UnknownPlannerError)
import pytest

# Modules only these planners import (the training module the UNet comes from needs pandas)
NEEDS = {'ara-star': ('networkx',), 'neural-network': ('torch', 'skimage', 'pandas')}


def _missing(name):
    return [m for m in NEEDS.get(name, ()) if importlib.util.find_spec(m) is None]


def test_default_is_dijkstra_and_listed():
    assert DEFAULT_PLANNER == 'dijkstra'
    assert DEFAULT_PLANNER in PLANNER_NAMES
    assert len(set(PLANNER_NAMES)) == len(PLANNER_NAMES) == 5


def test_launch_default_matches_the_registry():
    # launch_node_definitions.py keeps a literal default so it never imports this package
    launch_file = (Path(__file__).resolve().parents[4] / 'launches'
                   / 'launch_node_definitions.py')
    assert launch_file.exists(), f'{launch_file} not found; update this test if it moved'
    defaults = re.findall(r"LaunchConfiguration\('planner', default='([^']+)'\)",
                          launch_file.read_text())
    assert defaults == [DEFAULT_PLANNER]


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
    missing = _missing(name)
    if missing:
        # Heavy dependencies are optional; without one, picking that planner must fail loudly
        with pytest.raises(ImportError, match='|'.join(missing)):
            make_planner(name)
        return
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
