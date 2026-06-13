"""Unit tests for costs/pedestrian_costmap.py — no ROS spin required."""

import numpy as np

from costs.pedestrian_costmap import distance_to_cost

D_MAX = 10.0


class TestDistanceToCost:
    def test_in_road_is_max_cost(self):
        # distance 0 (pedestrian in the road) -> full cost
        assert distance_to_cost(0.0, D_MAX) == 100

    def test_at_dmax_is_zero(self):
        assert distance_to_cost(D_MAX, D_MAX) == 0

    def test_beyond_dmax_is_clipped_to_zero(self):
        assert distance_to_cost(D_MAX + 5.0, D_MAX) == 0

    def test_midrange_is_proportional(self):
        assert distance_to_cost(5.0, 10.0) == 50

    def test_degenerate_dmax_is_safe(self):
        assert distance_to_cost(1.0, 0.0) == 0


from costs.pedestrian_costmap import pose_to_grid_coords

GRID_SIZE = 151
RESOLUTION = 0.4
ORIGIN_X = -20.0
ORIGIN_Y = -30.0


def grid_coords(pos_x, pos_y):
    return pose_to_grid_coords(pos_x, pos_y, ORIGIN_X, ORIGIN_Y,
                               RESOLUTION, GRID_SIZE)


class TestPoseToGridCoords:
    def test_ego_origin_maps_to_center_cell(self):
        # (0,0) -> col = (0 - -20)/0.4 = 50 ; row = (0 - -30)/0.4 = 75
        assert grid_coords(0.0, 0.0) == (75, 50)

    def test_ahead_increases_col(self):
        row, col = grid_coords(10.0, 0.0)
        assert col > 50 and row == 75

    def test_behind_decreases_col(self):
        row, col = grid_coords(-10.0, 0.0)
        assert col < 50 and row == 75

    def test_left_increases_row(self):
        row, col = grid_coords(0.0, 10.0)
        assert row > 75 and col == 50

    def test_right_decreases_row(self):
        row, col = grid_coords(0.0, -10.0)
        assert row < 75 and col == 50

    def test_off_grid_returns_none(self):
        assert grid_coords(1000.0, 0.0) is None
        assert grid_coords(0.0, 1000.0) is None
