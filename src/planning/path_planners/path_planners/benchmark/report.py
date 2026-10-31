"""
Write calls.csv, summary.md and per-scenario plots from benchmark runs.

Package:   path_planners
Filename:  benchmark/report.py
Author:    Shifat Islam Santo

Turns run results into calls.csv (every call), summary.md (one row per
planner and scenario, plus a per-planner scorecard) and one PNG per scenario
with every planner's published (smoothed) path over the padded costmap.
"""

import csv
import os
import statistics
from typing import Dict, List

import numpy as np

from path_planners.planner_registry import PLANNER_NAMES
from path_planners.planning_core import rolling_smoothing

NODE_PERIOD_MS = 100.0  # path_planner_node replans on a 0.1 s timer
SANE_COST_RATIO = 2.0  # a valid path costing more than this x the optimum is not usable

# Categorical slots 1-5, fixed order by planner (validated: CVD and normal-vision
# separation pass on a light surface; legend + table carry identity)
PLANNER_COLORS = {
    'dijkstra': '#2a78d6',
    'ara-star': '#eb6834',
    'dynamic-programming': '#1baf7a',
    'neural-network': '#eda100',
    'trrt-star': '#e87ba4',
}
INK = '#1a1a19'
SURFACE = '#fcfcfb'

CALL_FIELDS = ['planner', 'scenario', 'call', 'status', 'runtime_ms', 'valid',
               'returned_path', 'starts_at_start', 'ends_at_goal', 'clear', 'clear_smoothed',
               'centre_hits_obstacle', 'first_too_close_m', 'enters_padding',
               'enters_padding_smoothed', 'cost', 'cost_ratio', 'length_m', 'min_clearance_m',
               'max_turn_deg', 'total_turn_deg', 'points', 'max_rss_mb', 'error']


def outcome(row: Dict, reachable: bool) -> str:
    """Describe what a call did in a short phrase, judged against the reference."""
    if row['status'] in ('timeout', 'error'):
        return row['status']
    if not row.get('returned_path'):
        return 'correct no path' if not reachable else 'no path'
    if row.get('valid'):
        return 'valid'
    if row.get('centre_hits_obstacle'):
        return 'hits obstacle'
    if not row.get('clear'):
        return 'too close'
    if not row.get('clear_smoothed'):
        return 'too close after smoothing'
    if not row.get('ends_at_goal'):
        return 'misses goal'
    return 'bad start'


def is_correct(word: str) -> bool:
    return word in ('valid', 'correct no path')


def write_calls_csv(rows: List[Dict], out_dir: str):
    with open(os.path.join(out_dir, 'calls.csv'), 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=CALL_FIELDS, extrasaction='ignore')
        writer.writeheader()
        for row in rows:
            writer.writerow({k: _fmt(row.get(k)) for k in CALL_FIELDS})


def _fmt(v):
    if isinstance(v, float):
        return f'{v:.4g}'
    return '' if v is None else v


def _num(v, spec='.1f', none='-'):
    return none if v is None else format(v, spec)


def summarize(rows: List[Dict], scenarios, planners, timeout_ms: float) -> Dict:
    """
    Aggregate per (planner, scenario) and per planner.

    A timed-out call counts as taking the timeout, a lower bound, so a planner
    that hangs cannot look fast; write_summary_md prints such values as '>'.
    """
    cells, scorecard = {}, {}
    for planner in planners:
        correct, usable, ratios, all_ms, under_budget, completed = 0, 0, [], [], 0, 0
        consistent, timeouts = 0, 0
        for sc in scenarios:
            mine = [r for r in rows if r['planner'] == planner and r['scenario'] == sc.name]
            if not mine:
                continue
            reachable = mine[0]['reachable']
            words = [outcome(r, reachable) for r in mine]
            ms = [r['runtime_ms'] if r.get('runtime_ms') is not None else timeout_ms
                  for r in mine if r.get('runtime_ms') is not None or r['status'] == 'timeout']
            valid = [r for r in mine if r.get('valid')]
            rss = [r['max_rss_mb'] for r in mine if r.get('max_rss_mb') is not None]
            first_valid = valid[0] if valid else (mine[0] if mine[0].get('returned_path') else {})
            cells[(planner, sc.name)] = {
                'first': words[0],
                'correct_calls': sum(is_correct(w) for w in words),
                'calls': len(words),
                'first_ms': (timeout_ms if mine[0]['status'] == 'timeout'
                             else mine[0].get('runtime_ms')),
                'median_ms': statistics.median(ms) if ms else None,
                'cost_ratio': first_valid.get('cost_ratio'),
                'length_m': first_valid.get('length_m'),
                'min_clearance_m': first_valid.get('min_clearance_m'),
                'max_turn_deg': first_valid.get('max_turn_deg'),
                'rss_growth_mb': (rss[-1] - rss[0]) if len(rss) > 1 else None,
                'error': next((r['error'] for r in mine if r.get('error')), None),
            }
            correct += is_correct(words[0])
            consistent += all(is_correct(w) for w in words)
            timeouts += sum(r['status'] == 'timeout' for r in mine)
            usable += is_correct(words[0]) and (
                words[0] != 'valid' or (mine[0].get('cost_ratio') or 0.0) <= SANE_COST_RATIO)
            if words[0] == 'valid' and mine[0].get('cost_ratio') is not None:
                ratios.append(mine[0]['cost_ratio'])
            all_ms.extend(ms)
            under_budget += sum(m <= NODE_PERIOD_MS for m in ms)
            completed += len(mine)
        scorecard[planner] = {
            'correct': correct,
            'consistent': consistent,
            'timeouts': timeouts,
            'usable': usable,
            'median_cost_ratio': statistics.median(ratios) if ratios else None,
            'scenarios': len(scenarios),
            'median_ms': statistics.median(all_ms) if all_ms else None,
            'p95_ms': float(np.percentile(all_ms, 95)) if all_ms else None,
            'under_budget': under_budget,
            'calls': completed,
        }
    return {'cells': cells, 'scorecard': scorecard, 'timeout_ms': timeout_ms}


def write_summary_md(summary, scenarios, planners, meta: Dict, out_dir: str):
    lines = ['# Path planner benchmark', '']
    lines += [f'- {k}: {v}' for k, v in meta.items()]
    t_ms = summary['timeout_ms']

    def ms(v):
        return f'>{t_ms:.0f}' if v is not None and v >= t_ms else _num(v)

    lines += ['', '## Scorecard', '',
              'Correct = valid path when one exists, no path when none exists, judged on the '
              'first call; all calls = correct on every back-to-back call, so state a planner '
              f'carries between calls shows. Usable = correct, and a valid path costs at most '
              f'{SANE_COST_RATIO:g}x the exact optimum. Budget = the node\'s 100 ms replanning '
              'period; a timed-out call counts as the timeout, so > marks a lower bound.', '',
              'Cost is measured under the objective the costmap encodes, (cell value + 1) per '
              'step, which is also what the stack\'s Dijkstra minimizes, so Dijkstra scores 1.00 '
              'by construction. The other planners optimize their own objectives; a ratio above '
              '1 means the path is costlier under the costmap\'s objective, not that it is '
              'invalid.', '',
              '| planner | correct | all calls | usable | median cost (valid) | median ms | '
              'p95 ms | timeouts | calls within 100 ms |',
              '|---|---|---|---|---|---|---|---|---|']
    for p in planners:
        s = summary['scorecard'][p]
        lines.append(f"| {p} | {s['correct']}/{s['scenarios']} | "
                     f"{s['consistent']}/{s['scenarios']} | {s['usable']}/{s['scenarios']} | "
                     f"{_num(s['median_cost_ratio'], '.2f')} | {ms(s['median_ms'])} | "
                     f"{ms(s['p95_ms'])} | {s['timeouts']} | "
                     f"{s['under_budget']}/{s['calls']} |")
    lines += ['', '## Per scenario', '',
              'cost = path cost / exact optimum under the same ruler (1.00 is optimal). '
              'Length, clearance and turning are measured on the smoothed path the node '
              'publishes. Clearance is to the nearest unpadded obstacle.', '']
    for sc in scenarios:
        lines += [f'### {sc.name}', '', sc.description, '',
                  f"Reference: {'path exists' if sc.reachable else 'no path exists'}"
                  + (f", exact cost {sc.ref_cost:.0f}, {sc.ref_length_m:.1f} m"
                     if sc.reachable else '')
                  + ('. Goal snapped out of an obstacle.' if sc.goal_snapped else '.'), '',
                  f'![{sc.name}]({sc.name}.png)', '',
                  '| planner | first call | correct calls | first ms | median ms | cost | '
                  'length m | clearance m | max turn deg | RSS growth MB |',
                  '|---|---|---|---|---|---|---|---|---|---|']
        for p in planners:
            c = summary['cells'].get((p, sc.name))
            if c is None:
                continue
            lines.append(
                f"| {p} | {c['first']} | {c['correct_calls']}/{c['calls']} | "
                f"{ms(c['first_ms'])} | {ms(c['median_ms'])} | "
                f"{_num(c['cost_ratio'], '.2f')} | {_num(c['length_m'])} | "
                f"{_num(c['min_clearance_m'], '.2f')} | {_num(c['max_turn_deg'], '.0f')} | "
                f"{_num(c['rss_growth_mb'], '.1f')} |")
        errors = [(p, summary['cells'][(p, sc.name)]['error']) for p in planners
                  if (p, sc.name) in summary['cells'] and summary['cells'][(p, sc.name)]['error']]
        for p, err in errors:
            last = err.strip().splitlines()[-1]
            lines.append(f'\n{p} error: `{last}`')
        lines.append('')
    with open(os.path.join(out_dir, 'summary.md'), 'w') as f:
        f.write('\n'.join(lines))


def plot_scenario(sc, rows: List[Dict], planners, out_dir: str):
    """Plot every planner's first published path over the padded costmap, in metres."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.patheffects as pe
    import matplotlib.pyplot as plt

    res = sc.resolution
    n_rows, n_cols = sc.problem.costmap.shape
    x0, y0 = -sc.origin_x, -sc.origin_y
    extent = (x0, x0 + n_cols * res, y0, y0 + n_rows * res)

    def to_xy(points):
        pts = np.asarray(points, dtype=float)
        return pts[:, 1] * res + x0, pts[:, 0] * res + y0

    fig, ax = plt.subplots(figsize=(9.6, 6.0), dpi=130)
    fig.patch.set_facecolor(SURFACE)
    ax.imshow(sc.problem.costmap, origin='lower', extent=extent, cmap='Greys',
              vmin=0, vmax=140, interpolation='nearest')
    ring = [pe.Stroke(linewidth=4, foreground=SURFACE), pe.Normal()]
    seen_x, seen_y = [], []
    if sc.ref_path is not None:
        xs, ys = to_xy(sc.ref_path)
        seen_x.extend(xs)
        seen_y.extend(ys)
        # Drawn last-on-top so a planner that matches it cannot hide it
        ax.plot(xs, ys, color=INK, linewidth=1.2, linestyle=(0, (4, 3)), zorder=5,
                label='exact reference')
    for p in planners:
        mine = [r for r in rows if r['planner'] == p and r['scenario'] == sc.name]
        if not mine:
            continue
        r = mine[0]
        word = outcome(r, sc.reachable)
        label = f'{p} ({word}'
        if word == 'valid' and r.get('cost_ratio') is not None:
            label += f", cost {r['cost_ratio']:.2f}x"
        label += ')'
        if r.get('path'):
            xs, ys = to_xy(rolling_smoothing([tuple(q) for q in r['path']]))
            seen_x.extend(xs)
            seen_y.extend(ys)
            ax.plot(xs, ys, color=PLANNER_COLORS[p], linewidth=2, label=label,
                    path_effects=ring)
        else:
            ax.plot([], [], color=PLANNER_COLORS[p], linewidth=2, label=label)
    sx, sy = to_xy([sc.problem.start])
    gx, gy = to_xy([sc.problem.goal])
    ax.plot(sx, sy, marker='o', markersize=8, color=INK, linestyle='none', label='start')
    ax.plot(gx, gy, marker='*', markersize=12, color=INK, linestyle='none', label='goal')
    # Frame everything drawn plus 5 m, never less than a 30 m x 24 m window
    seen_x.extend([sx[0], gx[0]])
    seen_y.extend([sy[0], gy[0]])
    cx, cy = (min(seen_x) + max(seen_x)) / 2, (min(seen_y) + max(seen_y)) / 2
    half_w = max(15.0, (max(seen_x) - min(seen_x)) / 2 + 5.0)
    half_h = max(12.0, (max(seen_y) - min(seen_y)) / 2 + 5.0)
    ax.set_xlim(max(extent[0], cx - half_w), min(extent[1], cx + half_w))
    ax.set_ylim(max(extent[2], cy - half_h), min(extent[3], cy + half_h))
    ax.set_xlabel('x forward (m, base_link)', color=INK)
    ax.set_ylabel('y left (m)', color=INK)
    ax.set_title(f'{sc.name}: dark = obstacle + 1 m padding, light band = route corridor',
                 color=INK, loc='left', fontsize=9)
    ax.tick_params(colors='#5f5e5a')
    for spine in ax.spines.values():
        spine.set_color('#c3c2b7')
    ax.legend(loc='upper left', bbox_to_anchor=(1.02, 1.0), fontsize=8, frameon=False,
              labelcolor=INK)
    fig.tight_layout()
    fig.savefig(os.path.join(out_dir, f'{sc.name}.png'), facecolor=SURFACE)
    plt.close(fig)


def ordered(planners):
    return [p for p in PLANNER_NAMES if p in planners]
