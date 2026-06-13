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
