"""Unit tests for lidar_ground_height.py — pure numpy, no ROS required."""

import os
import sys

import numpy as np
import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from segmentation import lidar_ground_height as lgh
from segmentation.bev_geometry import GRID_SIZE, RESOLUTION, ORIGIN_X, ORIGIN_Y, VEHICLE_ROW, VEHICLE_COL


def test_ground_height_grid_empty_input():
    grid = lgh.ground_height_grid(None, GRID_SIZE)
    assert grid.shape == (GRID_SIZE, GRID_SIZE)
    assert np.isnan(grid).all()

    grid2 = lgh.ground_height_grid(np.zeros((0, 4)), GRID_SIZE)
    assert np.isnan(grid2).all()


def test_ground_height_grid_reports_flat_road_and_curb_step():
    # Road at z=0.0 for x in [0, 10), sidewalk at z=0.15 (15cm curb) for x in [10, 15)
    xs = np.tile(np.arange(0.0, 15.0, 0.05), 1)
    ys = np.zeros_like(xs)
    zs = np.where(xs < 10.0, 0.0, 0.15)
    intensity = np.zeros_like(xs)
    points = np.stack([xs, ys, zs, intensity], axis=1)

    grid = lgh.ground_height_grid(points, GRID_SIZE)

    road_col = int((5.0 - ORIGIN_X) / RESOLUTION)
    curb_col = int((12.0 - ORIGIN_X) / RESOLUTION)
    row = VEHICLE_ROW
    assert grid[row, road_col] == pytest.approx(0.0, abs=0.01)
    assert grid[row, curb_col] == pytest.approx(0.15, abs=0.01)


def test_ground_height_grid_untrusted_below_min_hits():
    # Only a single point in one cell -- below MIN_HITS=2, must stay NaN.
    points = np.array([[0.0, 0.0, 0.0, 0.0]])
    grid = lgh.ground_height_grid(points, GRID_SIZE)
    assert np.isnan(grid).all()


if __name__ == '__main__':
    sys.exit(pytest.main([__file__, '-v']))
