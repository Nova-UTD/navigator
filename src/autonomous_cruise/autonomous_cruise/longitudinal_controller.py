#!/usr/bin/env python3
"""
Adaptive PID longitudinal controller for speed control.

Author: Siddarth Nandyala
Contact: siddarth.nandyala@utdallas.edu
Organization: Nova UTD Autonomous Driving
Date: November 2025
License: MIT
"""

import math
import numpy as np
from typing import Optional, Tuple
from navigator_msgs.msg import Object3DArray, Object3D


class AdaptiveLongitudinalController:
    """
    Adaptive longitudinal controller combining PID speed control with
    adaptive cruise control for safe vehicle following.
    """

    def __init__(
        self,
        target_speed: float = 6.0,
        kp: float = 1.2,
        ki: float = 0.15,
        kd: float = 0.05,
        max_accel: float = 2.5,
        max_decel: float = -3.5,
        max_jerk: float = 3.0,
        max_throttle: float = 0.75,
        max_brake: float = 1.0,
        integral_limit: float = 5.0,
        time_gap: float = 1.5,
        min_following_distance: float = 5.0,
        max_detection_distance: float = 50.0,
        detection_angle: float = 0.3
    ):
        """
        Initialize the adaptive longitudinal controller.

        Args:
            target_speed: Desired cruise speed (m/s).
            kp: Proportional gain for PID.
            ki: Integral gain for PID.
            kd: Derivative gain for PID.
            max_accel: Maximum acceleration (m/s²).
            max_decel: Maximum deceleration (m/s²).
            max_jerk: Maximum jerk for comfort (m/s³).
            max_throttle: Maximum throttle value (0-1).
            max_brake: Maximum brake value (0-1).
            integral_limit: Anti-windup limit for integral term.
            time_gap: Desired time gap to lead vehicle (seconds).
            min_following_distance: Minimum following distance (meters).
            max_detection_distance: Maximum distance to consider objects (meters).
            detection_angle: Half-angle of detection cone (radians).
        """
        # Speed control parameters
        self.target_speed = target_speed
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_jerk = max_jerk
        self.max_throttle = max_throttle
        self.max_brake = max_brake
        self.integral_limit = integral_limit

        # Adaptive cruise parameters
        self.time_gap = time_gap
        self.min_following_distance = min_following_distance
        self.max_detection_distance = max_detection_distance
        self.detection_angle = detection_angle

        # PID state
        self.integral = 0.0
        self.prev_err = 0.0
        self.last_a = 0.0

        # Lead vehicle tracking
        self.lead_vehicle: Optional[Object3D] = None
        self.lead_distance: float = 0.0
        self.lead_velocity: float = 0.0

    def compute_control(
        self,
        current_speed: float,
        objects: Optional[Object3DArray],
        dt: float
    ) -> Tuple[float, float, float]:
        """
        Compute throttle and brake commands with adaptive cruise control.

        Args:
            current_speed: Current vehicle speed (m/s).
            objects: Detected objects from perception.
            dt: Time step since last update (seconds).

        Returns:
            Tuple of (throttle, brake, target_speed).
            throttle: Throttle command (0-max_throttle).
            brake: Brake command (0-max_brake).
            target_speed: Adjusted target speed (m/s).
        """
        # Detect lead vehicle
        self._detect_lead_vehicle(objects, current_speed)

        # Compute adaptive target speed
        adaptive_target_speed = self._compute_adaptive_target_speed(
            current_speed
        )

        # PID control to track target speed
        a_cmd = self._compute_pid_acceleration(
            current_speed, adaptive_target_speed, dt
        )

        # Apply jerk limiting
        a_cmd = self._apply_jerk_limit(a_cmd, dt)

        # Apply acceleration limits
        a_cmd = self._apply_acceleration_limits(a_cmd)

        # Convert to throttle/brake
        throttle, brake = self._acceleration_to_throttle_brake(
            a_cmd, current_speed
        )

        return throttle, brake, adaptive_target_speed

    def _detect_lead_vehicle(
        self,
        objects: Optional[Object3DArray],
        current_speed: float
    ):
        """
        Detect the lead vehicle in front of the ego vehicle.

        Args:
            objects: Array of detected objects.
            current_speed: Current vehicle speed (m/s).
        """
        self.lead_vehicle = None
        self.lead_distance = 0.0
        self.lead_velocity = 0.0

        if objects is None or len(objects.objects) == 0:
            return

        min_distance = float('inf')
        lead_candidate = None

        for obj in objects.objects:
            # Only consider cars
            if obj.label != Object3D.CAR:
                continue

            # Get object position (center of bounding box)
            center_x = obj.bounding_box.center.position.x
            center_y = obj.bounding_box.center.position.y
            center_z = obj.bounding_box.center.position.z

            # Compute distance
            distance = math.sqrt(center_x**2 + center_y**2 + center_z**2)

            # Check if object is in detection range
            if distance > self.max_detection_distance:
                continue

            # Check if object is in front (within detection cone)
            angle = math.atan2(abs(center_y), center_x)
            if angle > self.detection_angle:
                continue

            # Check if object is ahead (positive x in vehicle frame)
            if center_x < 0:
                continue

            # Track closest object in front
            if distance < min_distance:
                min_distance = distance
                lead_candidate = obj

        if lead_candidate is not None:
            self.lead_vehicle = lead_candidate
            self.lead_distance = min_distance

            # Estimate lead vehicle velocity (simplified)
            # In a real system, this would come from the tracker
            # For now, assume similar speed or use object velocity if available
            self.lead_velocity = current_speed

    def _compute_adaptive_target_speed(self, current_speed: float) -> float:
        """
        Compute adaptive target speed based on lead vehicle.

        Args:
            current_speed: Current vehicle speed (m/s).

        Returns:
            Adjusted target speed (m/s).
        """
        if self.lead_vehicle is None:
            # No lead vehicle, use cruise speed
            return self.target_speed

        # Compute desired following distance
        desired_distance = (
            self.min_following_distance +
            self.time_gap * current_speed
        )

        # Compute distance error
        distance_error = self.lead_distance - desired_distance

        # Proportional control for distance tracking
        # If too close, reduce speed; if too far, increase speed
        k_distance = 0.5  # Gain for distance control
        speed_adjustment = k_distance * distance_error

        # Adjust target speed
        adaptive_speed = min(
            self.target_speed,
            self.lead_velocity + speed_adjustment
        )

        # Ensure non-negative speed
        adaptive_speed = max(0.0, adaptive_speed)

        return adaptive_speed

    def _compute_pid_acceleration(
        self,
        current_speed: float,
        target_speed: float,
        dt: float
    ) -> float:
        """
        Compute acceleration command using PID control.

        Args:
            current_speed: Current vehicle speed (m/s).
            target_speed: Target speed to track (m/s).
            dt: Time step (seconds).

        Returns:
            Acceleration command (m/s²).
        """
        # Speed error
        err = target_speed - current_speed

        # Integral term with anti-windup
        self.integral += err * dt
        self.integral = np.clip(
            self.integral,
            -self.integral_limit,
            self.integral_limit
        )

        # Derivative term
        if dt > 0:
            deriv = (err - self.prev_err) / dt
        else:
            deriv = 0.0

        self.prev_err = err

        # PID output
        a_cmd = self.kp * err + self.ki * self.integral + self.kd * deriv

        return a_cmd

    def _apply_jerk_limit(self, a_cmd: float, dt: float) -> float:
        """
        Apply jerk limiting for passenger comfort.

        Args:
            a_cmd: Desired acceleration (m/s²).
            dt: Time step (seconds).

        Returns:
            Jerk-limited acceleration (m/s²).
        """
        if dt <= 0:
            return a_cmd

        max_da = self.max_jerk * dt
        a_limited = np.clip(
            a_cmd,
            self.last_a - max_da,
            self.last_a + max_da
        )

        self.last_a = a_limited
        return a_limited

    def _apply_acceleration_limits(self, a_cmd: float) -> float:
        """
        Clamp acceleration to vehicle limits.

        Args:
            a_cmd: Desired acceleration (m/s²).

        Returns:
            Clamped acceleration (m/s²).
        """
        return np.clip(a_cmd, self.max_decel, self.max_accel)

    def _acceleration_to_throttle_brake(
        self,
        a_cmd: float,
        current_speed: float
    ) -> Tuple[float, float]:
        """
        Convert acceleration command to throttle and brake values.

        Args:
            a_cmd: Acceleration command (m/s²).
            current_speed: Current vehicle speed (m/s).

        Returns:
            Tuple of (throttle, brake).
        """
        deadband = 0.1  # m/s²

        if a_cmd > deadband:
            # Positive acceleration -> throttle
            throttle = (a_cmd / self.max_accel) * self.max_throttle
            throttle = np.clip(throttle, 0.0, self.max_throttle)
            brake = 0.0

        elif a_cmd < -deadband:
            # Negative acceleration -> brake
            throttle = 0.0
            brake = abs(a_cmd / self.max_decel) * self.max_brake
            brake = np.clip(brake, 0.0, self.max_brake)

        else:
            # Within deadband -> coast
            throttle = 0.0
            brake = 0.0

        return throttle, brake

    def reset(self):
        """Reset the controller state."""
        self.integral = 0.0
        self.prev_err = 0.0
        self.last_a = 0.0
        self.lead_vehicle = None
        self.lead_distance = 0.0
        self.lead_velocity = 0.0
