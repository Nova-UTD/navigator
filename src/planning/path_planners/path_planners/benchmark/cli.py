"""
Command line entry point for the planner benchmark.

Package:   path_planners
Filename:  benchmark/cli.py
Author:    Shifat Islam Santo

    python3 -m path_planners.benchmark [--planners dijkstra,trrt-star]
        [--scenarios parked_car,u_trap] [--costmaps DIR_OR_NPZ ...]
        [--repeats 3] [--timeout 15] [--jobs 1] [--nn-model PATH] [--out DIR]

Inside the navigator container: ros2 run path_planners planner_benchmark ...
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
import os
import platform
import sys
import time
from typing import List, Optional

from path_planners.benchmark import metrics, report, runner, scenarios
from path_planners.planner_registry import PLANNER_NAMES
from path_planners.planning_core import (
    build_problem, costmap_to_array, OBSTACLE_PADDING, OBSTACLE_THRESHOLD, ORIGIN_X,
    ORIGIN_Y, PlanningProblem)


@dataclass
class Prepared:
    """A scenario run through the node pipeline, with its exact reference."""

    name: str
    description: str
    resolution: float
    origin_x: float
    origin_y: float
    problem: PlanningProblem
    raw: object  # rescaled, unpadded costmap
    goal_snapped: bool
    reachable: bool
    ref_path: Optional[list]
    ref_cost: Optional[float]
    ref_length_m: Optional[float]
    clearance: object


def prepare(sc: scenarios.Scenario) -> Optional[Prepared]:
    raw = costmap_to_array(sc.costmap.ravel(), *sc.costmap.shape, sc.resolution)
    problem = build_problem(raw, sc.resolution, sc.goal_xy[0], sc.goal_xy[1],
                            ORIGIN_X, ORIGIN_Y, OBSTACLE_THRESHOLD, OBSTACLE_PADDING)
    if problem is None:
        print(f'  {sc.name}: goal blocked with no free cell back to start; the node '
              'would publish nothing. Skipped.')
        return None
    ref = metrics.reference_path(problem.costmap, problem.start, problem.goal,
                                 OBSTACLE_THRESHOLD)
    ref_cost = ref_len = None
    if ref is not None:
        pts = metrics.as_points(ref)
        ref_cost = metrics.path_cost(pts, problem.costmap)
        ref_len = metrics.length_cells(pts) * sc.resolution
    if sc.expect_reachable is not None and sc.expect_reachable != (ref is not None):
        print(f'  WARNING {sc.name}: scenario expects reachable={sc.expect_reachable} '
              f'but the exact reference says {ref is not None}')
    return Prepared(sc.name, sc.description, sc.resolution, ORIGIN_X, ORIGIN_Y, problem,
                    raw, problem.goal_snapped, ref is not None, ref, ref_cost, ref_len,
                    metrics.obstacle_distance(raw, OBSTACLE_THRESHOLD))


def parse_args(argv):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--planners', default='all',
                    help=f"comma list or 'all' ({', '.join(PLANNER_NAMES)})")
    ap.add_argument('--scenarios', default='all',
                    help="comma list of synthetic scenarios, 'all', or 'none'")
    ap.add_argument('--costmaps', nargs='*', default=[],
                    help='recorded .npz frames or directories of them')
    ap.add_argument('--repeats', type=int, default=3,
                    help='back-to-back calls per planner instance (default 3)')
    ap.add_argument('--timeout', type=float, default=15.0,
                    help='seconds allowed per call before it counts as a timeout')
    ap.add_argument('--jobs', type=int, default=1,
                    help='runs in parallel; keep 1 when the timings matter')
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--nn-model', default=None,
                    help='UNet checkpoint for neural-network (default: none, as the node runs)')
    ap.add_argument('--out', default=None, help='output directory')
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    planners = list(PLANNER_NAMES) if args.planners == 'all' else args.planners.split(',')
    bad = [p for p in planners if p not in PLANNER_NAMES]
    if bad:
        sys.exit(f"Unknown planners {bad}. Valid: {', '.join(PLANNER_NAMES)}")
    planners = report.ordered(planners)
    synthetic = (None if args.scenarios == 'all'
                 else [] if args.scenarios == 'none' else args.scenarios.split(','))
    out = args.out or os.path.join(os.getcwd(), time.strftime('planner_bench_%Y%m%d_%H%M%S'))
    os.makedirs(out, exist_ok=True)

    print('Preparing scenarios through the node pipeline...')
    prepared: List[Prepared] = [p for p in map(prepare, scenarios.collect(
        synthetic, args.costmaps)) if p is not None]
    for p in prepared:
        print(f"  {p.name}: {'path exists' if p.reachable else 'no path exists'}"
              + (f', exact {p.ref_length_m:.1f} m' if p.reachable else ''))

    options = {'neural-network': {'model_path': args.nn_model}}
    jobs = [(pl, sc) for sc in prepared for pl in planners]

    def run_one(job):
        pl, sc = job
        t0 = time.time()
        res = runner.run(pl, sc.name, sc.problem, OBSTACLE_THRESHOLD, args.repeats,
                         args.timeout, args.seed, options.get(pl))
        statuses = (','.join(c.status for c in res.calls)
                    or (res.error or 'error').strip().splitlines()[-1])
        print(f'  {pl:20s} {sc.name:22s} {time.time() - t0:6.1f} s  {statuses}', flush=True)
        return res

    print(f'Running {len(jobs)} planner x scenario runs, {args.repeats} calls each...')
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(run_one, jobs))

    by_name = {p.name: p for p in prepared}
    rows = []
    for res in results:
        sc = by_name[res.scenario]
        calls = res.calls or [runner.CallResult(0, 'error', error=res.error)]
        for call in calls:
            row = {'planner': res.planner, 'scenario': res.scenario, 'call': call.call,
                   'status': call.status, 'runtime_ms': call.runtime_ms,
                   'max_rss_mb': call.max_rss_mb, 'error': call.error,
                   'reachable': sc.reachable, 'path': call.path}
            if call.status in ('ok', 'no_path'):
                row.update(metrics.evaluate(call.path, sc.problem, OBSTACLE_THRESHOLD,
                                            sc.resolution, sc.clearance, sc.ref_cost,
                                            OBSTACLE_PADDING))
            rows.append(row)

    summary = report.summarize(rows, prepared, planners, args.timeout * 1e3)
    meta = {
        'date': time.strftime('%Y-%m-%d %H:%M %Z'),
        'host': f'{platform.node()} ({platform.processor() or platform.machine()}, '
                f'python {platform.python_version()})',
        'calls per run': args.repeats,
        'timeout per call': f'{args.timeout:.0f} s',
        'parallel jobs': args.jobs,
        'seed': args.seed,
        'neural-network weights': args.nn_model or 'none (untrained UNet, as the node runs today)',
        'node settings': f'threshold {OBSTACLE_THRESHOLD}, padding {OBSTACLE_PADDING} cells',
        'required clearance': f'{metrics.required_clearance_m(OBSTACLE_PADDING, 0.2):.2f} m '
                              'at 0.2 m cells (vehicle half-width less half a cell)',
    }
    report.write_calls_csv(rows, out)
    report.write_summary_md(summary, prepared, planners, meta, out)
    for sc in prepared:
        report.plot_scenario(sc, rows, planners, out)

    print('\nScorecard (first call correct, usable, median ms):')
    for p in planners:
        s = summary['scorecard'][p]
        print(f"  {p:20s} {s['correct']:2d}/{s['scenarios']}  {s['usable']:2d}/{s['scenarios']}"
              f"  {report._num(s['median_ms'])} ms")
    print(f'\nWrote {out}/summary.md, calls.csv and {len(prepared)} plots')
    return 0


if __name__ == '__main__':
    sys.exit(main())
