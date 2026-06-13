"""Unit tests for pedestrian_geometry — no ROS / ML deps required."""

from pedestrian_intent_to_enter_road.pedestrian_geometry import (
    estimate_depth, project_to_base_link,
)

IMG_W = 672   # ZED 720p capture width (~672x376)


class TestEstimateDepth:
    def test_closer_pedestrian_has_larger_bbox_smaller_is_farther(self):
        # depth = f * H / h_px  -> taller bbox (more px) = nearer = smaller depth
        assert estimate_depth(200.0) < estimate_depth(100.0)

    def test_known_value(self):
        # 470 * 1.75 / 100 = 8.225 m
        assert abs(estimate_depth(100.0) - 8.225) < 1e-6

    def test_zero_or_negative_height_is_safe(self):
        assert estimate_depth(0.0) == 0.0
        assert estimate_depth(-5.0) == 0.0


class TestProjectToBaseLink:
    def test_centered_pedestrian_has_zero_lateral(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W)
        assert abs(pos_y) < 1e-6

    def test_right_of_center_is_negative_y(self):
        # center_x > cx  ->  vehicle's right  ->  negative pos_y
        _, pos_y = project_to_base_link(IMG_W / 2.0 + 100.0, 100.0, IMG_W)
        assert pos_y < 0.0

    def test_left_of_center_is_positive_y(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0 - 100.0, 100.0, IMG_W)
        assert pos_y > 0.0

    def test_pos_x_is_depth_plus_offset(self):
        pos_x, _ = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W,
                                        cam_offset_x=1.5)
        assert abs(pos_x - (estimate_depth(100.0) + 1.5)) < 1e-6

    def test_pos_y_offset_applied(self):
        _, pos_y = project_to_base_link(IMG_W / 2.0, 100.0, IMG_W,
                                        cam_offset_y=0.5)
        assert abs(pos_y - 0.5) < 1e-6
