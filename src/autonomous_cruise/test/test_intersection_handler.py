"""
Unit tests for intersection handler.

Tests the intersection state machine, smooth stopping, and safety checks.
"""

import unittest
import sys
import os

# Add parent directory to path for imports
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from autonomous_cruise.intersection_handler import (
    IntersectionHandler,
    IntersectionState
)
from navigator_msgs.msg import IntersectionBehavior
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TestIntersectionHandler(unittest.TestCase):
    """Test cases for IntersectionHandler."""

    def setUp(self):
        """Set up test fixtures."""
        self.handler = IntersectionHandler(
            approach_speed=3.0,
            crossing_speed=4.0,
            exit_speed=5.0,
            comfortable_decel=-1.5,
            min_stop_duration=1.0,
            proceed_delay=0.5
        )

    def test_initial_state(self):
        """Test initial state is CRUISING."""
        self.assertEqual(self.handler.state, IntersectionState.CRUISING)

    def test_transition_to_approaching(self):
        """Test transition to APPROACHING_INTERSECTION."""
        # Create "Wait" behavior
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        
        self.handler.update_intersection_behavior(behavior)
        
        # Create pose
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        
        # Update state
        self.handler.update_state(pose, 5.0, 0.0)
        
        # Should transition to approaching
        self.assertEqual(
            self.handler.state,
            IntersectionState.APPROACHING_INTERSECTION
        )

    def test_transition_to_waiting(self):
        """Test transition to WAITING_AT_INTERSECTION."""
        # Set up approaching state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        
        # Now stop
        self.handler.update_state(pose, 0.0, 1.0)
        
        # Should transition to waiting
        self.assertEqual(
            self.handler.state,
            IntersectionState.WAITING_AT_INTERSECTION
        )

    def test_transition_to_crossing(self):
        """Test transition to CROSSING_INTERSECTION."""
        # Set up waiting state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        self.handler.update_state(pose, 0.0, 1.0)
        
        # Change to "Proceed"
        behavior.action = "Proceed"
        self.handler.update_intersection_behavior(behavior)
        
        # Wait for minimum stop duration + proceed delay
        self.handler.update_state(pose, 0.0, 2.6)
        
        # Should transition to crossing
        self.assertEqual(
            self.handler.state,
            IntersectionState.CROSSING_INTERSECTION
        )

    def test_smooth_stop_deceleration(self):
        """Test smooth stop deceleration calculation."""
        # Set up approaching state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        
        # Compute deceleration
        decel = self.handler.compute_intersection_deceleration(5.0, pose)
        
        # Should return a deceleration value
        self.assertIsNotNone(decel)
        self.assertLess(decel, 0.0)  # Should be negative (deceleration)
        self.assertGreaterEqual(decel, -2.5)  # Within limits

    def test_target_speed_approaching(self):
        """Test target speed when approaching intersection."""
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        
        # Target speed should be 0 (stopping)
        target = self.handler.get_target_speed(cruise_speed=5.0)
        self.assertEqual(target, 0.0)

    def test_target_speed_crossing(self):
        """Test target speed when crossing intersection."""
        # Set up crossing state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        self.handler.update_state(pose, 0.0, 1.0)
        
        behavior.action = "Proceed"
        self.handler.update_intersection_behavior(behavior)
        self.handler.update_state(pose, 0.0, 2.6)
        
        # Target speed should be crossing speed
        target = self.handler.get_target_speed(cruise_speed=5.0)
        self.assertEqual(target, 4.0)  # crossing_speed

    def test_hold_brake_when_waiting(self):
        """Test brake hold when waiting at intersection."""
        # Set up waiting state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        self.handler.update_state(pose, 0.0, 1.0)
        
        # Should hold brake
        self.assertTrue(self.handler.should_hold_brake())

    def test_no_brake_hold_when_cruising(self):
        """Test no brake hold when cruising."""
        # Initial cruising state
        self.assertFalse(self.handler.should_hold_brake())

    def test_state_name(self):
        """Test state name retrieval."""
        self.assertEqual(self.handler.get_state_name(), "CRUISING")
        
        # Change state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        
        self.assertEqual(self.handler.get_state_name(), "APPROACHING_INTERSECTION")

    def test_minimum_stop_duration(self):
        """Test minimum stop duration enforcement."""
        # Set up waiting state
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        self.handler.update_intersection_behavior(behavior)
        
        pose = Pose()
        self.handler.update_state(pose, 5.0, 0.0)
        self.handler.update_state(pose, 0.0, 1.0)
        
        # Immediately change to "Proceed"
        behavior.action = "Proceed"
        self.handler.update_intersection_behavior(behavior)
        
        # Update at 0.5 seconds (less than min_stop_duration)
        self.handler.update_state(pose, 0.0, 1.5)
        
        # Should still be waiting
        self.assertEqual(
            self.handler.state,
            IntersectionState.WAITING_AT_INTERSECTION
        )
        
        # Update at 2.0 seconds (more than min_stop_duration + proceed_delay)
        self.handler.update_state(pose, 0.0, 3.1)
        
        # Should now be crossing
        self.assertEqual(
            self.handler.state,
            IntersectionState.CROSSING_INTERSECTION
        )


class TestIntersectionScenarios(unittest.TestCase):
    """Test complete intersection scenarios."""

    def test_traffic_light_red_to_green(self):
        """Test traffic light scenario: red -> green."""
        handler = IntersectionHandler()
        
        pose = Pose()
        pose.position.x = 0.0
        pose.position.y = 0.0
        
        # Red light - "Wait"
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        handler.update_intersection_behavior(behavior)
        
        # Approaching at 5 m/s
        handler.update_state(pose, 5.0, 0.0)
        self.assertEqual(handler.state, IntersectionState.APPROACHING_INTERSECTION)
        
        # Decelerate and stop
        handler.update_state(pose, 2.0, 1.0)
        handler.update_state(pose, 0.0, 2.0)
        self.assertEqual(handler.state, IntersectionState.WAITING_AT_INTERSECTION)
        
        # Light turns green - "Proceed"
        behavior.action = "Proceed"
        handler.update_intersection_behavior(behavior)
        
        # Wait minimum duration
        handler.update_state(pose, 0.0, 3.6)
        self.assertEqual(handler.state, IntersectionState.CROSSING_INTERSECTION)
        
        # Cross intersection
        pose.position.x = 20.0  # Moved 20m
        handler.update_state(pose, 4.0, 8.0)
        self.assertEqual(handler.state, IntersectionState.EXITING_INTERSECTION)

    def test_stop_sign_scenario(self):
        """Test 4-way stop sign scenario."""
        handler = IntersectionHandler(min_stop_duration=1.0)
        
        pose = Pose()
        
        # Approaching stop sign
        behavior = IntersectionBehavior()
        behavior.action = "Wait"
        handler.update_intersection_behavior(behavior)
        
        handler.update_state(pose, 5.0, 0.0)
        
        # Stop at stop sign
        handler.update_state(pose, 0.0, 2.0)
        self.assertEqual(handler.state, IntersectionState.WAITING_AT_INTERSECTION)
        
        # Wait for turn (intersection manager checks other cars)
        handler.update_state(pose, 0.0, 3.0)
        handler.update_state(pose, 0.0, 4.0)
        
        # Our turn - "Proceed"
        behavior.action = "Proceed"
        handler.update_intersection_behavior(behavior)
        
        # Proceed after delay
        handler.update_state(pose, 0.0, 4.6)
        self.assertEqual(handler.state, IntersectionState.CROSSING_INTERSECTION)


if __name__ == '__main__':
    unittest.main()
