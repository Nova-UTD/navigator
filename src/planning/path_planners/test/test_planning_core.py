"""Tests for planning_core, the node's costmap -> planning problem pipeline."""

import numpy as np
from path_planners.planning_core import (
    build_problem, cells_to_xy, costmap_to_array, OBSTACLE_PADDING, OBSTACLE_THRESHOLD,
    ORIGIN_X, ORIGIN_Y, pad_obstacles, snap_goal, xy_to_cell)
import pytest
import scipy.ndimage


def _legacy_pipeline(data, raw_h, raw_w, resolution, goal_x, goal_y):
    """path_planner_node.generate_path before the refactor, copied verbatim (minus logging)."""
    origin_x, origin_y, threshold, padding = 20.0, 30.0, 90, 5
    target_cells = int(60.0 / resolution)
    costmap_np = np.asarray(data, dtype=np.int32).reshape(raw_h, raw_w)
    if raw_h != target_cells or raw_w != target_cells:
        old = np.array(data).reshape(raw_h, raw_w)
        costmap_np = scipy.ndimage.zoom(
            old, (target_cells / raw_h, target_cells / raw_w), order=0).astype(np.int32)
    height, width = costmap_np.shape
    start_i = int(round(origin_y / resolution))
    start_j = int(round(origin_x / resolution))
    goal_i = int(round((goal_y + origin_y) / resolution))
    goal_j = int(round((goal_x + origin_x) / resolution))
    start_i = max(0, min(start_i, height - 1))
    start_j = max(0, min(start_j, width - 1))
    goal_i = max(0, min(goal_i, height - 1))
    goal_j = max(0, min(goal_j, width - 1))

    obstacles = 0 * costmap_np
    obstacles[np.where(costmap_np >= threshold)] = 1
    struct2 = scipy.ndimage.generate_binary_structure(2, 2)
    obstacles = scipy.ndimage.binary_dilation(
        obstacles, structure=struct2, iterations=padding).astype(costmap_np.dtype)
    padded = costmap_np.copy()
    padded[obstacles > 0] = 100

    if padded[goal_i, goal_j] >= threshold:
        steps = max(abs(goal_i - start_i), abs(goal_j - start_j))
        if steps > 0:
            snapped = False
            for s in range(steps, -1, -1):
                t = s / steps
                ci = int(round(start_i + t * (goal_i - start_i)))
                cj = int(round(start_j + t * (goal_j - start_j)))
                ci = max(0, min(ci, height - 1))
                cj = max(0, min(cj, width - 1))
                if padded[ci, cj] < threshold:
                    goal_i, goal_j = ci, cj
                    snapped = True
                    break
            if not snapped:
                return None
    return padded, (start_i, start_j), (goal_i, goal_j)


def test_constants_match_the_node_before_refactor():
    assert (ORIGIN_X, ORIGIN_Y, OBSTACLE_THRESHOLD, OBSTACLE_PADDING) == (20.0, 30.0, 90, 5)


@pytest.mark.parametrize('shape,resolution', [((300, 300), 0.2), ((151, 151), 0.4),
                                              ((200, 260), 0.2)])
def test_build_problem_matches_legacy_node_pipeline(shape, resolution):
    rng = np.random.default_rng(7)
    outcomes = {'free': 0, 'snapped': 0, 'blocked': 0}
    for trial in range(60):
        grid = rng.choice([0, 10, 50], size=shape, p=[0.5, 0.2, 0.3])
        for _ in range(rng.integers(0, 12)):  # sparse rectangular obstacles
            r, c = rng.integers(0, shape[0]), rng.integers(0, shape[1])
            grid[r:r + rng.integers(2, 60), c:c + rng.integers(2, 60)] = 100
        if trial % 10 == 9:  # a wall across the whole grid
            grid[:, rng.integers(shape[1] // 2, shape[1])] = 100
        if trial % 10 == 4:  # vehicle boxed in: every cell back to the start blocked
            grid[:, :] = 100
        goal = (rng.uniform(-25, 45), rng.uniform(-35, 35))  # includes off-grid goals
        legacy = _legacy_pipeline(grid.ravel().tolist(), *shape, resolution, *goal)
        costmap = costmap_to_array(grid.ravel().tolist(), *shape, resolution)
        new = build_problem(costmap, resolution, goal[0], goal[1], ORIGIN_X, ORIGIN_Y,
                            OBSTACLE_THRESHOLD, OBSTACLE_PADDING)
        if legacy is None:
            assert new is None, trial
            outcomes['blocked'] += 1
            continue
        assert new is not None, trial
        np.testing.assert_array_equal(new.costmap, legacy[0])
        assert new.start == legacy[1] and new.goal == legacy[2], trial
        outcomes['snapped' if new.goal_snapped else 'free'] += 1
    # The comparison is only worth something if every branch was exercised
    assert all(n > 0 for n in outcomes.values()), outcomes


def test_published_grid_needs_no_rescale_and_starts_at_vehicle():
    grid = np.zeros((300, 300), dtype=np.int32)
    out = costmap_to_array(grid.ravel(), 300, 300, 0.2)
    assert out.shape == (300, 300)
    assert xy_to_cell(0.0, 0.0, 0.2, ORIGIN_X, ORIGIN_Y, out.shape) == (150, 100)


def test_cell_xy_round_trip():
    for x, y in [(0.0, 0.0), (12.4, -3.2), (-19.8, 29.6)]:
        cell = xy_to_cell(x, y, 0.2, ORIGIN_X, ORIGIN_Y, (300, 300))
        (bx, by), = cells_to_xy([cell], 0.2, ORIGIN_X, ORIGIN_Y)
        assert abs(bx - x) < 0.1 + 1e-9 and abs(by - y) < 0.1 + 1e-9


def test_pad_obstacles_is_a_square_of_padding_cells():
    grid = np.zeros((31, 31), dtype=np.int32)
    grid[15, 15] = 100
    padded = pad_obstacles(grid, 90, 5)
    rows, cols = np.nonzero(padded == 100)
    assert (rows.min(), rows.max(), cols.min(), cols.max()) == (10, 20, 10, 20)
    assert (padded == 100).sum() == 11 * 11


def test_snap_goal():
    grid = np.zeros((20, 20), dtype=np.int32)
    assert snap_goal(grid, (10, 0), (10, 15), 90) == (10, 15)  # free goal untouched
    grid[:, 12:18] = 100
    assert snap_goal(grid, (10, 0), (10, 15), 90) == (10, 11)  # nearest free cell back
    grid[:, :] = 100
    assert snap_goal(grid, (10, 0), (10, 15), 90) is None  # whole line blocked
    assert snap_goal(grid, (10, 0), (10, 0), 90) == (10, 0)  # legacy: returned as is
