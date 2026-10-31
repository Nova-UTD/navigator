"""Tests for the offline planner benchmark: the ruler, the scenarios and the runner."""

import os
import time

import numpy as np
from path_planners.benchmark import cli, metrics, report, runner, scenarios
from path_planners.planner_registry import make_planner, Planner
from path_planners.planning_core import PlanningProblem
import pytest


def _problem(costmap, start, goal):
    return PlanningProblem(costmap, start, goal, False)


def test_reference_path_cost_on_a_known_grid():
    costmap = np.zeros((5, 5), dtype=np.int32)
    path = metrics.reference_path(costmap, (0, 0), (0, 4), 90)
    assert path[0] == (0, 0) and path[-1] == (0, 4) and len(path) == 5
    costmap[:, 2] = 100
    assert metrics.reference_path(costmap, (0, 0), (0, 4), 90) is None
    costmap[4, 2] = 0  # one gap at the bottom
    path = metrics.reference_path(costmap, (0, 0), (0, 4), 90)
    assert (4, 2) in path


def test_path_cost_is_the_dijkstra_edge_cost_on_grid_paths():
    costmap = np.full((3, 6), 9, dtype=np.int32)
    straight = metrics.densify(metrics.as_points([(1, c) for c in range(6)]))
    assert metrics.path_cost(straight, costmap) == pytest.approx(5 * (9 + 1))


def test_rasterize_is_8_connected_and_hits_both_ends():
    cells = metrics.rasterize(np.array([[0, 0], [3, 7], [10, 2]], dtype=float))
    assert tuple(cells[0]) == (0, 0) and tuple(cells[-1]) == (10, 2)
    assert (3, 7) in map(tuple, cells)
    assert np.abs(np.diff(cells, axis=0)).max() == 1


def test_no_path_scores_below_the_reference():
    rng = np.random.default_rng(3)
    for _ in range(25):
        costmap = rng.choice([0, 10, 50], size=(40, 40)).astype(np.int32)
        costmap[rng.integers(0, 40, 30), rng.integers(0, 40, 30)] = 100
        start, goal = (2, 2), (37, 37)
        costmap[start] = costmap[goal] = 0
        ref = metrics.reference_path(costmap, start, goal, 90)
        if ref is None:
            continue
        ref_cost = metrics.path_cost(metrics.as_points(ref), costmap)
        # Any free path, including straight shortcuts and the planners' own output
        dijkstra = make_planner('dijkstra')(costmap, start, goal, 90)
        candidates = [ref[::3] + [ref[-1]], dijkstra] if dijkstra else [ref[::3] + [ref[-1]]]
        for path in candidates:
            cells = metrics.rasterize(metrics.as_points(path))
            if (costmap[cells[:, 0], cells[:, 1]] >= 90).any():
                continue  # the guarantee is over free paths; this one cuts an obstacle
            assert metrics.path_cost(metrics.as_points(path), costmap) >= ref_cost - 1e-9


def test_diagonal_steps_do_not_touch_corner_cells():
    costmap = np.zeros((4, 4), dtype=np.int32)
    costmap[1, 1] = costmap[2, 2] = 100  # both corners of the (1,2)->(2,1) step
    # The step's midpoint (1.5, 1.5) would round onto blocked (2, 2)
    pts = metrics.densify(metrics.as_points([(0, 3), (1, 2), (2, 1), (3, 0)]))
    assert metrics.first_collision(pts, costmap, 90) is None


def test_evaluate_valid_too_close_and_missed_goal():
    costmap = np.zeros((40, 40), dtype=np.int32)
    costmap[30:, 20] = 100
    clearance = metrics.obstacle_distance(costmap, 90)
    problem = _problem(costmap, (10, 2), (10, 38))
    ref = metrics.reference_path(costmap, (10, 2), (10, 38), 90)
    ref_cost = metrics.path_cost(metrics.as_points(ref), costmap)

    good = metrics.evaluate([(10, c) for c in range(2, 39)], problem, 90, 0.2,
                            clearance, ref_cost, 5)
    assert good['valid'] and good['cost_ratio'] == pytest.approx(1.0)
    assert good['min_clearance_m'] == pytest.approx(20 * 0.2)

    close = metrics.evaluate([(10, 2), (28, 20), (10, 38)], problem, 90, 0.2,
                             clearance, ref_cost, 5)
    assert not close['valid'] and not close['clear']

    short = metrics.evaluate([(10, c) for c in range(2, 30)], problem, 90, 0.2,
                             clearance, ref_cost, 5)
    assert not short['valid'] and not short['ends_at_goal']

    empty = metrics.evaluate(None, problem, 90, 0.2, clearance, ref_cost, 5)
    assert empty == {'returned_path': False}


def test_turning():
    assert metrics.turning_deg(np.array([[0, 0], [0, 1], [0, 2]], float)) == (0.0, 0.0)
    max_turn, total = metrics.turning_deg(np.array([[0, 0], [0, 1], [1, 1]], float))
    assert max_turn == pytest.approx(90.0) and total == pytest.approx(90.0)


@pytest.mark.parametrize('name', list(scenarios.SYNTHETIC))
def test_scenarios_match_their_declared_reachability(name):
    prepared = cli.prepare(scenarios.SYNTHETIC[name]())
    sc = scenarios.SYNTHETIC[name]()
    assert prepared.problem.costmap.shape == (300, 300)
    assert prepared.problem.start == (150, 100)
    if sc.expect_reachable is not None:
        assert prepared.reachable == sc.expect_reachable


def test_goal_in_obstacle_scenario_snaps():
    assert cli.prepare(scenarios.SYNTHETIC['goal_in_obstacle']()).goal_snapped


def test_recorded_frames_round_trip(tmp_path):
    grid = scenarios.SYNTHETIC['parked_car']().costmap
    np.savez(tmp_path / 'frame_0001.npz', costmap=grid, resolution=0.2, goal_x=25.0,
             goal_y=0.0, name='bag_frame_0001')
    loaded = scenarios.collect([], [str(tmp_path)])
    assert [s.name for s in loaded] == ['bag_frame_0001']
    np.testing.assert_array_equal(loaded[0].costmap, grid)


def test_runner_records_a_timeout_and_keeps_going(monkeypatch):
    def slow(name, **options):
        def plan(costmap, start, goal, threshold):
            time.sleep(5)
        return Planner(name, None, plan)
    monkeypatch.setattr(runner, 'make_planner', slow)
    problem = _problem(np.zeros((10, 10), dtype=np.int32), (0, 0), (9, 9))
    t0 = time.time()
    res = runner.run('dijkstra', 'x', problem, 90, repeats=3, timeout_s=0.5)
    assert time.time() - t0 < 3
    assert [c.status for c in res.calls] == ['timeout'] * 3


def test_runner_reports_errors_per_call(monkeypatch):
    def broken(name, **options):
        def plan(costmap, start, goal, threshold):
            raise AttributeError('boom')
        return Planner(name, None, plan)
    monkeypatch.setattr(runner, 'make_planner', broken)
    problem = _problem(np.zeros((10, 10), dtype=np.int32), (0, 0), (9, 9))
    res = runner.run('dijkstra', 'x', problem, 90, repeats=2, timeout_s=5)
    assert [c.status for c in res.calls] == ['error', 'error']
    assert 'boom' in res.calls[0].error


def test_cli_end_to_end(tmp_path):
    out = tmp_path / 'bench'
    assert cli.main(['--planners', 'dijkstra', '--scenarios',
                     'parked_car,unreachable,detour_beyond_margin', '--repeats', '1',
                     '--out', str(out)]) == 0
    summary = (out / 'summary.md').read_text()
    assert '| dijkstra | 2/3 |' in summary  # the detour leaves its 40-cell search box
    for name in ('parked_car', 'unreachable', 'detour_beyond_margin'):
        assert os.path.exists(out / f'{name}.png')
    assert len((out / 'calls.csv').read_text().splitlines()) == 1 + 3  # header + 3 calls


@pytest.mark.parametrize('args,message', [
    (['--repeats', '0'], None),
    (['--timeout', '-1'], None),
    (['--jobs', '0'], None),
    (['--planners', 'bogus'], 'Unknown planners'),
    (['--scenarios', 'nosuch'], 'Unknown scenarios'),
    (['--scenarios', 'none'], 'Nothing to run'),
    (['--scenarios', 'none', '--costmaps', '/nonexistent'], 'No such file or directory'),
])
def test_cli_rejects_bad_input_before_writing(tmp_path, args, message):
    out = tmp_path / 'bench'
    with pytest.raises(SystemExit) as exc:
        cli.main(args + ['--out', str(out)])
    assert exc.value.code != 0
    if message:
        assert message in str(exc.value.code)
    assert not out.exists()


def test_cli_rejects_empty_and_foreign_frames(tmp_path):
    empty = tmp_path / 'empty'
    empty.mkdir()
    foreign = tmp_path / 'foreign.npz'
    np.savez(foreign, costmap=np.zeros((4, 4), dtype=np.int8))
    for item, message in ((empty, 'No .npz frames'), (foreign, 'missing resolution')):
        with pytest.raises(SystemExit) as exc:
            cli.main(['--scenarios', 'none', '--costmaps', str(item),
                      '--out', str(tmp_path / 'bench')])
        assert message in str(exc.value.code)


def test_timeouts_count_in_timing_and_consistency():
    sc = cli.prepare(scenarios.SYNTHETIC['parked_car']())
    rows = [{'planner': 'trrt-star', 'scenario': sc.name, 'reachable': True, 'status': s,
             'runtime_ms': ms, 'returned_path': s == 'ok', 'valid': s == 'ok'}
            for s, ms in (('ok', 50.0), ('timeout', None), ('timeout', None))]
    summary = report.summarize(rows, [sc], ['trrt-star'], timeout_ms=15000.0)
    card = summary['scorecard']['trrt-star']
    assert card['median_ms'] == 15000.0  # not the 50 ms of the one call that finished
    assert (card['correct'], card['consistent'], card['timeouts']) == (1, 0, 2)
