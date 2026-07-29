#!/usr/bin/env python3
"""
Pure Pursuit lateral controller for path following.

Author: Siddarth Nandyala
Contact: siddarth.nandyala@utdallas.edu
Organization: Nova UTD Autonomous Driving
Date: November 2025
License: MIT
"""

import math
import numpy as np
from typing import Optional, Tuple, List
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path


class PurePursuitController:
    """
    Pure Pursuit lateral controller for path following.

    The Pure Pursuit algorithm is a path tracking algorithm that calculates
    the steering angle needed to follow a path by looking ahead along the path
    and steering towards a target point.
    """

    def __init__(
        self,
        wheelbase: float = 2.875,
        min_lookahead: float = 3.0,
        max_lookahead: float = 15.0,
        lookahead_gain: float = 0.5,
        max_steer: float = 1.0
    ):
        """
        Initialize the Pure Pursuit controller.

        Args:
            wheelbase: Distance between front and rear axles (meters).
            min_lookahead: Minimum lookahead distance (meters).
            max_lookahead: Maximum lookahead distance (meters).
            lookahead_gain: Gain for speed-dependent lookahead (dimensionless).
            max_steer: Maximum steering angle (radians).
        """
        self.wheelbase = wheelbase
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead
        self.lookahead_gain = lookahead_gain
        self.max_steer = max_steer

        self.path: Optional[Path] = None
        self.current_pose: Optional[Pose] = None

    def set_path(self, path: Path):
        """
        Set the reference path to follow.

        Args:
            path: Path message containing waypoints.
        """
        self.path = path

    def compute_steering(
        self,
        current_pose: Pose,
        current_speed: float
    ) -> Tuple[float, float]:
        """
        Compute steering command using Pure Pursuit algorithm.

        Args:
            current_pose: Current vehicle pose.
            current_speed: Current vehicle speed (m/s).

        Returns:
            Tuple of (steering_angle, cross_track_error).
            steering_angle: Steering command in range [-max_steer, max_steer].
            cross_track_error: Distance from vehicle to path (meters).
        """
        if self.path is None or len(self.path.poses) == 0:
            return 0.0, 0.0

        self.current_pose = current_pose

        # Compute adaptive lookahead distance
        lookahead_dist = self._compute_lookahead_distance(current_speed)

        # Find the target point on the path
        target_point, target_idx = self._find_target_point(
            current_pose, lookahead_dist
        )

        if target_point is None:
            # No valid target point, return zero steering
            return 0.0, 0.0

        # Compute cross-track error (for monitoring)
        cross_track_error = self._compute_cross_track_error(current_pose)

        # Compute steering angle
        steering_angle = self._compute_pure_pursuit_steering(
            current_pose, target_point
        )

        # Clamp steering to limits
        steering_angle = np.clip(steering_angle, -self.max_steer, self.max_steer)

        return steering_angle, cross_track_error

    def _compute_lookahead_distance(self, speed: float) -> float:
        """
        Compute adaptive lookahead distance based on speed.

        Args:
            speed: Current vehicle speed (m/s).

        Returns:
            Lookahead distance (meters).
        """
        lookahead = self.min_lookahead + self.lookahead_gain * speed
        return np.clip(lookahead, self.min_lookahead, self.max_lookahead)

    def _find_target_point(
        self,
        current_pose: Pose,
        lookahead_dist: float
    ) -> Tuple[Optional[Point], int]:
        """
        Find the target point on the path at the lookahead distance.

        Args:
            current_pose: Current vehicle pose.
            lookahead_dist: Lookahead distance (meters).

        Returns:
            Tuple of (target_point, target_index).
            target_point: Point on path at lookahead distance.
            target_index: Index of the target point in the path.
        """
        if self.path is None:
            return None, -1

        current_x = current_pose.position.x
        current_y = current_pose.position.y

        # Find the closest point on the path
        min_dist = float('inf')
        closest_idx = 0

        for i, pose_stamped in enumerate(self.path.poses):
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < min_dist:
                min_dist = dist
                closest_idx = i

        # Search forward from closest point for lookahead point
        for i in range(closest_idx, len(self.path.poses)):
            pose_stamped = self.path.poses[i]
            dx = pose_stamped.pose.position.x - current_x
            dy = pose_stamped.pose.position.y - current_y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= lookahead_dist:
                return pose_stamped.pose.position, i

        # If no point found at lookahead distance, use last point
        if len(self.path.poses) > 0:
            return self.path.poses[-1].pose.position, len(self.path.poses) - 1

        return None, -1

    def _compute_pure_pursuit_steering(
        self,
        current_pose: Pose,
        target_point: Point
    ) -> float:
        """
        Compute steering angle using Pure Pursuit geometry.

        Args:
            current_pose: Current vehicle pose.
            target_point: Target point to steer towards.

        Returns:
            Steering angle (radians).
        """
        # Transform target point to vehicle frame
        dx = target_point.x - current_pose.position.x
        dy = target_point.y - current_pose.position.y

        # Get vehicle heading from quaternion
        yaw = self._quaternion_to_yaw(current_pose.orientation)

        # Rotate to vehicle frame
        cos_yaw = math.cos(-yaw)
        sin_yaw = math.sin(-yaw)
        target_x = dx * cos_yaw - dy * sin_yaw
        target_y = dx * sin_yaw + dy * cos_yaw

        # Compute lookahead distance
        ld = math.sqrt(target_x * target_x + target_y * target_y)

        if ld < 0.1:  # Avoid division by zero
            return 0.0

        # Pure Pursuit formula: steering = atan(2 * L * sin(alpha) / ld)
        # where alpha is the angle to the target point
        alpha = math.atan2(target_y, target_x)
        steering = math.atan2(2.0 * self.wheelbase * math.sin(alpha), ld)

        return steering

    def _compute_cross_track_error(self, current_pose: Pose) -> float:
        """
        Compute the cross-track error (distance from vehicle to path).

        Args:
            current_pose: Current vehicle pose.

        Returns:
            Cross-track error (meters).
        """
        if self.path is None or len(self.path.poses) < 2:
            return 0.0

        current_x = current_pose.position.x
        current_y = current_pose.position.y

        min_dist = float('inf')

        # Find minimum distance to any path segment
        for i in range(len(self.path.poses) - 1):
            p1 = self.path.poses[i].pose.position
            p2 = self.path.poses[i + 1].pose.position

            # Compute distance from point to line segment
            dist = self._point_to_segment_distance(
                current_x, current_y,
                p1.x, p1.y,
                p2.x, p2.y
            )

            min_dist = min(min_dist, dist)

        return min_dist

    @staticmethod
    def _point_to_segment_distance(
        px: float, py: float,
        x1: float, y1: float,
        x2: float, y2: float
    ) -> float:
        """
        Compute distance from point to line segment.

        Args:
            px, py: Point coordinates.
            x1, y1: Segment start coordinates.
            x2, y2: Segment end coordinates.

        Returns:
            Distance from point to segment (meters).
        """
        dx = x2 - x1
        dy = y2 - y1
        length_sq = dx * dx + dy * dy

        if length_sq < 1e-6:
            # Segment is a point
            return math.sqrt((px - x1) ** 2 + (py - y1) ** 2)

        # Project point onto line
        t = ((px - x1) * dx + (py - y1) * dy) / length_sq
        t = max(0.0, min(1.0, t))

        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy

        return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)

    @staticmethod
    def _quaternion_to_yaw(quaternion) -> float:
        """
        Convert quaternion to yaw angle.

        Args:
            quaternion: Quaternion message.

        Returns:
            Yaw angle (radians).
        """
        # Extract quaternion components
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Compute yaw
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return yaw
