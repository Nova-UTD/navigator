#!/usr/bin/env python3
"""
Intersection handling with traffic light and stop sign support.

Author: Siddarth Nandyala
Contact: siddarth.nandyala@utdallas.edu
Organization: Nova UTD Autonomous Driving
Date: November 2025
License: MIT
"""

import math
from enum import Enum
from typing import Optional, Tuple
from navigator_msgs.msg import IntersectionBehavior
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose


class IntersectionState(Enum):
    """States for intersection navigation."""
    CRUISING = 0
    APPROACHING_INTERSECTION = 1
    WAITING_AT_INTERSECTION = 2
    CROSSING_INTERSECTION = 3
    EXITING_INTERSECTION = 4


class IntersectionHandler:
    """
    Handles intersection navigation logic.
    
    Integrates with Navigator's intersection_manager to provide smooth,
    safe intersection crossing with proper stop/go behavior.
    """
    
    def __init__(
        self,
        approach_speed: float = 3.0,
        crossing_speed: float = 4.0,
        exit_speed: float = 5.0,
        comfortable_decel: float = -1.5,
        emergency_decel: float = -2.5,
        stop_line_buffer: float = 1.0,
        min_stop_duration: float = 1.0,
        proceed_delay: float = 0.5,
        intersection_clearance_distance: float = 15.0
    ):
        """
        Initialize intersection handler.
        
        Args:
            approach_speed: Speed when approaching intersection (m/s).
            crossing_speed: Speed when crossing intersection (m/s).
            exit_speed: Speed when exiting intersection (m/s).
            comfortable_decel: Comfortable deceleration rate (m/s²).
            emergency_decel: Emergency deceleration rate (m/s²).
            stop_line_buffer: Distance to stop before stop line (m).
            min_stop_duration: Minimum time to remain stopped (s).
            proceed_delay: Delay after "Proceed" before moving (s).
            intersection_clearance_distance: Distance to clear intersection (m).
        """
        # Speed parameters
        self.approach_speed = approach_speed
        self.crossing_speed = crossing_speed
        self.exit_speed = exit_speed
        
        # Deceleration parameters
        self.comfortable_decel = comfortable_decel
        self.emergency_decel = emergency_decel
        self.stop_line_buffer = stop_line_buffer
        
        # Timing parameters
        self.min_stop_duration = min_stop_duration
        self.proceed_delay = proceed_delay
        self.intersection_clearance_distance = intersection_clearance_distance
        
        # State
        self.state = IntersectionState.CRUISING
        self.intersection_behavior: Optional[str] = None
        self.stop_time: Optional[float] = None
        self.proceed_time: Optional[float] = None
        self.intersection_entry_pose: Optional[Pose] = None
        
        # Path tracking
        self.current_path: Optional[Path] = None
        self.stop_line_distance: Optional[float] = None
    
    def update_intersection_behavior(self, behavior: IntersectionBehavior):
        """
        Update intersection behavior from intersection manager.
        
        Args:
            behavior: IntersectionBehavior message ("Wait" or "Proceed").
        """
        self.intersection_behavior = behavior.action
    
    def set_path(self, path: Path):
        """
        Update the current path.
        
        Args:
            path: Path message.
        """
        self.current_path = path
    
    def update_state(
        self,
        current_pose: Pose,
        current_speed: float,
        current_time: float
    ):
        """
        Update intersection state machine.
        
        Args:
            current_pose: Current vehicle pose.
            current_speed: Current vehicle speed (m/s).
            current_time: Current time (seconds).
        """
        if self.intersection_behavior is None:
            # No intersection behavior - normal cruising
            self.state = IntersectionState.CRUISING
            return
        
        # State machine transitions
        if self.state == IntersectionState.CRUISING:
            if self.intersection_behavior == "Wait":
                # Intersection detected, need to stop
                self.state = IntersectionState.APPROACHING_INTERSECTION
                self.stop_line_distance = self._estimate_stop_line_distance(
                    current_pose
                )
        
        elif self.state == IntersectionState.APPROACHING_INTERSECTION:
            if current_speed < 0.1:
                # Vehicle has stopped
                self.state = IntersectionState.WAITING_AT_INTERSECTION
                self.stop_time = current_time
        
        elif self.state == IntersectionState.WAITING_AT_INTERSECTION:
            if self.intersection_behavior == "Proceed":
                # Check if we've stopped long enough
                time_stopped = current_time - (self.stop_time or current_time)
                if time_stopped >= self.min_stop_duration:
                    if self.proceed_time is None:
                        self.proceed_time = current_time
                    
                    # Wait for proceed delay
                    time_since_proceed = current_time - self.proceed_time
                    if time_since_proceed >= self.proceed_delay:
                        self.state = IntersectionState.CROSSING_INTERSECTION
                        self.intersection_entry_pose = current_pose
                        self.proceed_time = None
        
        elif self.state == IntersectionState.CROSSING_INTERSECTION:
            # Check if we've cleared the intersection
            if self.intersection_entry_pose is not None:
                distance_traveled = self._distance_between_poses(
                    self.intersection_entry_pose,
                    current_pose
                )
                if distance_traveled >= self.intersection_clearance_distance:
                    self.state = IntersectionState.EXITING_INTERSECTION
        
        elif self.state == IntersectionState.EXITING_INTERSECTION:
            # Check if we can resume normal cruising
            if self.intersection_behavior != "Wait":
                self.state = IntersectionState.CRUISING
                self._reset_intersection_state()
    
    def get_target_speed(self, cruise_speed: float) -> float:
        """
        Get target speed based on intersection state.
        
        Args:
            cruise_speed: Normal cruise speed (m/s).
        
        Returns:
            Target speed for current intersection state (m/s).
        """
        if self.state == IntersectionState.APPROACHING_INTERSECTION:
            return 0.0  # Target stop
        elif self.state == IntersectionState.WAITING_AT_INTERSECTION:
            return 0.0  # Remain stopped
        elif self.state == IntersectionState.CROSSING_INTERSECTION:
            return self.crossing_speed
        elif self.state == IntersectionState.EXITING_INTERSECTION:
            return self.exit_speed
        else:
            return cruise_speed
    
    def compute_intersection_deceleration(
        self,
        current_speed: float,
        current_pose: Pose
    ) -> Optional[float]:
        """
        Compute smooth deceleration for intersection stopping.
        
        Args:
            current_speed: Current vehicle speed (m/s).
            current_pose: Current vehicle pose.
        
        Returns:
            Deceleration command (m/s²), or None if not applicable.
        """
        if self.state != IntersectionState.APPROACHING_INTERSECTION:
            return None
        
        if current_speed < 0.1:
            # Already stopped
            return 0.0
        
        # Estimate distance to stop line
        distance_to_stop = self._estimate_stop_line_distance(current_pose)
        
        if distance_to_stop is None or distance_to_stop < 0.5:
            # Very close or unknown - emergency stop
            return self.emergency_decel
        
        # Compute required deceleration: v² = v₀² + 2ad
        # Solving for a: a = -v₀² / (2d)
        required_decel = -(current_speed ** 2) / (2 * distance_to_stop)
        
        # Clamp to comfortable range
        decel = max(required_decel, self.emergency_decel)
        decel = min(decel, self.comfortable_decel)
        
        return decel
    
    def should_hold_brake(self) -> bool:
        """
        Check if vehicle should hold brake (at intersection).
        
        Returns:
            True if brake should be held.
        """
        return self.state == IntersectionState.WAITING_AT_INTERSECTION
    
    def is_safe_to_proceed(self) -> bool:
        """
        Check if it's safe to proceed through intersection.
        
        Returns:
            True if safe to proceed.
        """
        if self.state != IntersectionState.CROSSING_INTERSECTION:
            return False
        
        # Check intersection manager says proceed
        if self.intersection_behavior != "Proceed":
            return False
        
        return True
    
    def _estimate_stop_line_distance(self, current_pose: Pose) -> Optional[float]:
        """
        Estimate distance to stop line.
        
        Args:
            current_pose: Current vehicle pose.
        
        Returns:
            Estimated distance to stop line (m), or None if unknown.
        """
        if self.stop_line_distance is not None:
            # Use cached value if available
            return self.stop_line_distance
        
        if self.current_path is None or len(self.current_path.poses) < 10:
            # No path or too short - use conservative estimate
            return 15.0  # meters
        
        # Find a point ahead in the path (likely near stop line)
        # Look 10-30 meters ahead
        current_x = current_pose.position.x
        current_y = current_pose.position.y
        
        min_distance = 10.0
        max_distance = 30.0
        
        for pose_stamped in self.current_path.poses:
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            distance = math.sqrt(dx * dx + dy * dy)
            
            if min_distance <= distance <= max_distance:
                # This is likely near the stop line
                return distance - self.stop_line_buffer
        
        # Fallback: use middle of range
        return 20.0
    
    def _distance_between_poses(self, pose1: Pose, pose2: Pose) -> float:
        """
        Compute distance between two poses.
        
        Args:
            pose1: First pose.
            pose2: Second pose.
        
        Returns:
            Distance (m).
        """
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        return math.sqrt(dx * dx + dy * dy)
    
    def _reset_intersection_state(self):
        """Reset intersection-specific state variables."""
        self.stop_time = None
        self.proceed_time = None
        self.intersection_entry_pose = None
        self.stop_line_distance = None
    
    def get_state_name(self) -> str:
        """
        Get human-readable state name.
        
        Returns:
            State name string.
        """
        return self.state.name
