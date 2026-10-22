"""
Run one planner on one problem in a forked, time-limited process.

Package:   path_planners
Filename:  benchmark/runner.py
Author:    Shifat Islam Santo

Runs one planner on one problem the way path_planner_node does: one planner
instance, called repeatedly on the same costmap, so state a planner carries
between calls shows up here too. Each run lives in a forked process so a
planner that hangs can be stopped at the timeout without stopping the
benchmark, and its memory growth can be read from that process alone.
"""

from dataclasses import dataclass, field
import multiprocessing
import random
import resource
import sys
import time
import traceback
from typing import List, Optional

import numpy as np

from path_planners.planner_registry import make_planner


@dataclass
class CallResult:
    """One planner call: status is ok, no_path, error or timeout."""

    call: int
    status: str
    runtime_ms: Optional[float] = None
    path: Optional[list] = None
    max_rss_mb: Optional[float] = None
    error: Optional[str] = None


@dataclass
class RunResult:
    planner: str
    scenario: str
    construct_ms: Optional[float]
    calls: List[CallResult] = field(default_factory=list)
    error: Optional[str] = None


def _rss_mb():
    return resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0


def _worker(conn, planner_name, options, costmap, start, goal, threshold, repeats, seed):
    try:
        random.seed(seed)
        np.random.seed(seed)
        if planner_name == 'neural-network':
            import torch
            torch.manual_seed(seed)  # before the UNet is built, so it seeds the weights
        t0 = time.perf_counter()
        planner = make_planner(planner_name, **options)
        conn.send(('ready', (time.perf_counter() - t0) * 1e3))
        sys.stdout.flush()
    except Exception:
        conn.send(('fatal', traceback.format_exc()))
        return
    for k in range(repeats):
        try:
            t0 = time.perf_counter()
            path = planner(costmap.copy(), start, goal, threshold)
            dt = (time.perf_counter() - t0) * 1e3
            if path is None or len(path) == 0:
                conn.send(('call', CallResult(k, 'no_path', dt, None, _rss_mb())))
            else:
                pts = [(float(p[0]), float(p[1])) for p in path]
                conn.send(('call', CallResult(k, 'ok', dt, pts, _rss_mb())))
        except Exception:
            conn.send(('call', CallResult(k, 'error', None, None, _rss_mb(),
                                          traceback.format_exc(limit=4))))
        # The parent kills this process when it is done; keep planner prints
        sys.stdout.flush()


def run(planner_name, scenario_name, problem, threshold, repeats=3,
        timeout_s=15.0, seed=0, options=None) -> RunResult:
    """Run `repeats` back-to-back calls; each call gets `timeout_s` seconds."""
    ctx = multiprocessing.get_context('fork')
    parent, child = ctx.Pipe(duplex=False)
    proc = ctx.Process(target=_worker, daemon=True, args=(
        child, planner_name, options or {}, problem.costmap, problem.start,
        problem.goal, threshold, repeats, seed))
    proc.start()
    child.close()
    result = RunResult(planner_name, scenario_name, None)
    try:
        # Construction (e.g. loading torch) gets the same budget as a call
        if not parent.poll(timeout_s):
            result.error = f'construction exceeded {timeout_s:.0f} s'
            return result
        kind, payload = parent.recv()
        if kind == 'fatal':
            result.error = payload
            return result
        result.construct_ms = payload
        for k in range(repeats):
            if not parent.poll(timeout_s):
                result.calls.extend(CallResult(i, 'timeout') for i in range(k, repeats))
                break
            _, call = parent.recv()
            result.calls.append(call)
    except EOFError:
        result.error = 'planner process died (likely out of memory or a native crash)'
    finally:
        if proc.is_alive():
            proc.kill()
        proc.join(5)
        parent.close()
    return result
