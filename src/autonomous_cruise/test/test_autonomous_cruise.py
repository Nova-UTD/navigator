"""Unit tests for autonomous cruise controller."""

import unittest
import math
import numpy as np

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from navigator_msgs.msg import Object3D, Object3DArray, BoundingBox3D

from autonomous_cruise.lateral_controller import PurePursuitController
from autonomous_cruise.longitudinal_controller import AdaptiveLongitudinalController


class TestPurePursuitController(unittest.TestCase):
    """Test suite for Pure Pursuit lateral controller."""

    def setUp(self):
        """Set up test fixtures."""
        self.controller = PurePursuitController(
            wheelbase=2.875,
            min_lookahead=3.0,
            max_lookahead=15.0,
            lookahead_gain=0.5,
            max_steer=1.0
        )

    def test_initialization(self):
        """Test controller initializes correctly."""
        self.assertIsNotNone(self.controller)
        self.assertEqual(self.controller.wheelbase, 2.875)
        self.assertEqual(self.controller.min_lookahead, 3.0)

    def test_lookahead_distance_computation(self):
        """Test adaptive lookahead distance."""
        # At low speed
        ld_low = self.controller._compute_lookahead_distance(2.0)
        self.assertAlmostEqual(ld_low, 4.0, places=2)  # 3.0 + 0.5*2.0

        # At high speed
        ld_high = self.controller._compute_lookahead_distance(20.0)
        self.assertEqual(ld_high, 15.0)  # Clamped to max

    def test_steering_with_straight_path(self):
        """Test steering for straight path ahead."""
        # Create straight path
        path = Path()
        for i in range(10):
            pose = PoseStamped()
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.controller.set_path(path)

        # Vehicle at origin facing forward
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.orientation.w = 1.0

        steer, cte = self.controller.compute_steering(current_pose, 5.0)

        # Should have near-zero steering for straight path
        self.assertLess(abs(steer), 0.1)

    def test_steering_with_offset_path(self):
        """Test steering when path is offset to the side."""
        # Create path offset to the left
        path = Path()
        for i in range(10):
            pose = PoseStamped()
            pose.pose.position.x = float(i)
            pose.pose.position.y = 2.0  # Offset left
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.controller.set_path(path)

        # Vehicle at origin
        current_pose = Pose()
        current_pose.position.x = 0.0
        current_pose.position.y = 0.0
        current_pose.orientation.w = 1.0

        steer, cte = self.controller.compute_steering(current_pose, 5.0)

        # Should steer left (positive)
        self.assertGreater(steer, 0.0)
        # Cross-track error should be ~2.0
        self.assertAlmostEqual(cte, 2.0, delta=0.5)

    def test_quaternion_to_yaw(self):
        """Test quaternion to yaw conversion."""
        # Facing forward (yaw = 0)
        q = Quaternion()
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        yaw = PurePursuitController._quaternion_to_yaw(q)
        self.assertAlmostEqual(yaw, 0.0, places=5)

        # Facing left (yaw = π/2)
        q.w = math.cos(math.pi / 4)
        q.z = math.sin(math.pi / 4)
        yaw = PurePursuitController._quaternion_to_yaw(q)
        self.assertAlmostEqual(yaw, math.pi / 2, places=5)


class TestAdaptiveLongitudinalController(unittest.TestCase):
    """Test suite for adaptive longitudinal controller."""

    def setUp(self):
        """Set up test fixtures."""
        self.controller = AdaptiveLongitudinalController(
            target_speed=6.0,
            kp=1.2,
            ki=0.15,
            kd=0.05,
            max_accel=2.5,
            max_decel=-3.5,
            max_jerk=3.0,
            max_throttle=0.75,
            max_brake=1.0,
            integral_limit=5.0,
            time_gap=1.5,
            min_following_distance=5.0
        )

    def test_initialization(self):
        """Test controller initializes correctly."""
        self.assertIsNotNone(self.controller)
        self.assertEqual(self.controller.target_speed, 6.0)
        self.assertEqual(self.controller.kp, 1.2)

    def test_control_without_lead_vehicle(self):
        """Test speed control without lead vehicle."""
        current_speed = 3.0
        dt = 0.05

        throttle, brake, target_speed = self.controller.compute_control(
            current_speed, None, dt
        )

        # Should accelerate (speed below target)
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)
        # Target should be cruise speed
        self.assertEqual(target_speed, 6.0)

    def test_control_at_target_speed(self):
        """Test control when at target speed."""
        current_speed = 6.0
        dt = 0.05

        # Run a few iterations to stabilize
        for _ in range(10):
            throttle, brake, target_speed = self.controller.compute_control(
                current_speed, None, dt
            )

        # Should have minimal control effort
        self.assertLess(throttle, 0.2)
        self.assertLess(brake, 0.2)

    def test_lead_vehicle_detection(self):
        """Test lead vehicle detection."""
        # Create object array with a car ahead
        objects = Object3DArray()
        
        car = Object3D()
        car.label = Object3D.CAR
        car.id = 1
        car.confidence_score = 0.9
        
        # Car 10 meters ahead
        bbox = BoundingBox3D()
        bbox.center.position.x = 10.0
        bbox.center.position.y = 0.0
        bbox.center.position.z = 0.0
        car.bounding_box = bbox
        
        objects.objects.append(car)

        # Detect lead vehicle
        self.controller._detect_lead_vehicle(objects, 5.0)

        # Should detect the car
        self.assertIsNotNone(self.controller.lead_vehicle)
        self.assertAlmostEqual(self.controller.lead_distance, 10.0, places=1)

    def test_adaptive_speed_with_lead_vehicle(self):
        """Test adaptive speed adjustment with lead vehicle."""
        # Create lead vehicle
        objects = Object3DArray()
        car = Object3D()
        car.label = Object3D.CAR
        bbox = BoundingBox3D()
        bbox.center.position.x = 8.0  # Close (less than desired distance)
        bbox.center.position.y = 0.0
        bbox.center.position.z = 0.0
        car.bounding_box = bbox
        objects.objects.append(car)

        current_speed = 6.0
        dt = 0.05

        throttle, brake, target_speed = self.controller.compute_control(
            current_speed, objects, dt
        )

        # Should reduce target speed (too close to lead vehicle)
        self.assertLess(target_speed, 6.0)

    def test_jerk_limiting(self):
        """Test jerk limiting prevents abrupt changes."""
        self.controller.last_a = 0.0
        dt = 0.05

        # Request large acceleration change
        a_cmd = 10.0
        a_limited = self.controller._apply_jerk_limit(a_cmd, dt)

        # Should be limited by jerk
        max_change = self.controller.max_jerk * dt
        self.assertLessEqual(a_limited, max_change)

    def test_acceleration_limits(self):
        """Test acceleration clamping."""
        # Test upper limit
        a_cmd = 100.0
        a_limited = self.controller._apply_acceleration_limits(a_cmd)
        self.assertEqual(a_limited, self.controller.max_accel)

        # Test lower limit
        a_cmd = -100.0
        a_limited = self.controller._apply_acceleration_limits(a_cmd)
        self.assertEqual(a_limited, self.controller.max_decel)

    def test_throttle_brake_conversion(self):
        """Test acceleration to throttle/brake conversion."""
        # Positive acceleration -> throttle
        throttle, brake = self.controller._acceleration_to_throttle_brake(
            2.0, 5.0
        )
        self.assertGreater(throttle, 0.0)
        self.assertEqual(brake, 0.0)

        # Negative acceleration -> brake
        throttle, brake = self.controller._acceleration_to_throttle_brake(
            -2.0, 5.0
        )
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

        # Small acceleration -> coast
        throttle, brake = self.controller._acceleration_to_throttle_brake(
            0.05, 5.0
        )
        self.assertEqual(throttle, 0.0)
        self.assertEqual(brake, 0.0)


if __name__ == '__main__':
    unittest.main()
