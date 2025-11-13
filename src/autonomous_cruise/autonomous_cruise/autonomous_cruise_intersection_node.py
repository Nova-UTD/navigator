#!/usr/bin/env python3
"""
Complete autonomous cruise controller with intersection handling.

Author: Siddarth Nandyala
Contact: siddarth.nandyala@utdallas.edu
Organization: Nova UTD Autonomous Driving
Date: November 2025
License: MIT
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from nav_msgs.msg import Odometry, Path
from navigator_msgs.msg import VehicleControl, Object3DArray, IntersectionBehavior
from std_msgs.msg import String
from geometry_msgs.msg import Pose

import math
from typing import Optional

from autonomous_cruise.lateral_controller import PurePursuitController
from autonomous_cruise.longitudinal_controller import AdaptiveLongitudinalController
from autonomous_cruise.intersection_handler import IntersectionHandler, IntersectionState


class AutonomousCruiseIntersectionController(Node):
    """
    Autonomous cruise controller with intersection handling.

    Combines lateral control, longitudinal control, and intersection
    navigation for complete autonomous driving capability.
    """

    def __init__(self):
        """Initialize the autonomous cruise controller node."""
        super().__init__('autonomous_cruise_controller')

        # Declare parameters
        self._declare_parameters()

        # Load parameters
        self._load_parameters()

        # Initialize controllers
        self.lateral_controller = PurePursuitController(
            wheelbase=self.wheelbase,
            min_lookahead=self.min_lookahead,
            max_lookahead=self.max_lookahead,
            lookahead_gain=self.lookahead_gain,
            max_steer=self.max_steer
        )

        self.longitudinal_controller = AdaptiveLongitudinalController(
            target_speed=self.target_speed,
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            max_accel=self.max_accel,
            max_decel=self.max_decel,
            max_jerk=self.max_jerk,
            max_throttle=self.max_throttle,
            max_brake=self.max_brake,
            integral_limit=self.integral_limit,
            time_gap=self.time_gap,
            min_following_distance=self.min_following_distance,
            max_detection_distance=self.max_detection_distance,
            detection_angle=self.detection_angle
        )

        self.intersection_handler = IntersectionHandler(
            approach_speed=self.intersection_approach_speed,
            crossing_speed=self.intersection_crossing_speed,
            exit_speed=self.intersection_exit_speed,
            comfortable_decel=self.intersection_comfortable_decel,
            emergency_decel=self.intersection_emergency_decel,
            stop_line_buffer=self.intersection_stop_line_buffer,
            min_stop_duration=self.intersection_min_stop_duration,
            proceed_delay=self.intersection_proceed_delay,
            intersection_clearance_distance=self.intersection_clearance_distance
        )

        # State variables
        self.current_path: Optional[Path] = None
        self.current_odom: Optional[Odometry] = None
        self.current_objects: Optional[Object3DArray] = None
        self.last_control_time = self.get_clock().now()
        self.enabled = True

        # QoS profiles
        qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            qos_reliable
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            qos_best_effort
        )

        self.objects_sub = self.create_subscription(
            Object3DArray,
            self.objects_topic,
            self.objects_callback,
            qos_reliable
        )

        self.intersection_sub = self.create_subscription(
            IntersectionBehavior,
            self.intersection_topic,
            self.intersection_callback,
            qos_reliable
        )

        # Publishers
        self.control_pub = self.create_publisher(
            VehicleControl,
            self.control_topic,
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            '/autonomous_cruise/debug',
            10
        )

        self.intersection_state_pub = self.create_publisher(
            String,
            '/autonomous_cruise/intersection_state',
            10
        )

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info('Autonomous Cruise Controller (Intersection-Aware) initialized')
        self.get_logger().info(f'Target speed: {self.target_speed:.2f} m/s')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Intersection handling: ENABLED')

    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        # Topic names
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('odom_topic', '/gnss/odometry')
        self.declare_parameter('objects_topic', '/tracked/objects3d')
        self.declare_parameter('control_topic', '/vehicle/control')
        self.declare_parameter('intersection_topic', '/intersection')

        # Control parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('enabled', True)

        # Vehicle parameters
        self.declare_parameter('wheelbase', 3.404)

        # Lateral control parameters
        self.declare_parameter('min_lookahead', 2.5)
        self.declare_parameter('max_lookahead', 10.0)
        self.declare_parameter('lookahead_gain', 0.4)
        self.declare_parameter('max_steer', 0.8)

        # Longitudinal control parameters
        self.declare_parameter('target_speed', 5.0)
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.12)
        self.declare_parameter('kd', 0.08)
        self.declare_parameter('max_accel', 1.5)
        self.declare_parameter('max_decel', -2.5)
        self.declare_parameter('max_jerk', 2.0)
        self.declare_parameter('max_throttle', 0.70)
        self.declare_parameter('max_brake', 1.0)
        self.declare_parameter('integral_limit', 3.0)

        # Adaptive cruise parameters
        self.declare_parameter('time_gap', 2.0)
        self.declare_parameter('min_following_distance', 4.0)
        self.declare_parameter('max_detection_distance', 30.0)
        self.declare_parameter('detection_angle', 0.4)

        # Intersection handling parameters
        self.declare_parameter('intersection_approach_speed', 3.0)
        self.declare_parameter('intersection_crossing_speed', 4.0)
        self.declare_parameter('intersection_exit_speed', 5.0)
        self.declare_parameter('intersection_comfortable_decel', -1.5)
        self.declare_parameter('intersection_emergency_decel', -2.5)
        self.declare_parameter('intersection_stop_line_buffer', 1.0)
        self.declare_parameter('intersection_min_stop_duration', 1.0)
        self.declare_parameter('intersection_proceed_delay', 0.5)
        self.declare_parameter('intersection_clearance_distance', 15.0)

    def _load_parameters(self):
        """Load parameters from ROS2 parameter server."""
        # Topic names
        self.path_topic = self.get_parameter('path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.objects_topic = self.get_parameter('objects_topic').value
        self.control_topic = self.get_parameter('control_topic').value
        self.intersection_topic = self.get_parameter('intersection_topic').value

        # Control parameters
        self.control_rate = self.get_parameter('control_rate').value
        self.enabled = self.get_parameter('enabled').value

        # Vehicle parameters
        self.wheelbase = self.get_parameter('wheelbase').value

        # Lateral control parameters
        self.min_lookahead = self.get_parameter('min_lookahead').value
        self.max_lookahead = self.get_parameter('max_lookahead').value
        self.lookahead_gain = self.get_parameter('lookahead_gain').value
        self.max_steer = self.get_parameter('max_steer').value

        # Longitudinal control parameters
        self.target_speed = self.get_parameter('target_speed').value
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.max_accel = self.get_parameter('max_accel').value
        self.max_decel = self.get_parameter('max_decel').value
        self.max_jerk = self.get_parameter('max_jerk').value
        self.max_throttle = self.get_parameter('max_throttle').value
        self.max_brake = self.get_parameter('max_brake').value
        self.integral_limit = self.get_parameter('integral_limit').value

        # Adaptive cruise parameters
        self.time_gap = self.get_parameter('time_gap').value
        self.min_following_distance = self.get_parameter(
            'min_following_distance'
        ).value
        self.max_detection_distance = self.get_parameter(
            'max_detection_distance'
        ).value
        self.detection_angle = self.get_parameter('detection_angle').value

        # Intersection handling parameters
        self.intersection_approach_speed = self.get_parameter(
            'intersection_approach_speed'
        ).value
        self.intersection_crossing_speed = self.get_parameter(
            'intersection_crossing_speed'
        ).value
        self.intersection_exit_speed = self.get_parameter(
            'intersection_exit_speed'
        ).value
        self.intersection_comfortable_decel = self.get_parameter(
            'intersection_comfortable_decel'
        ).value
        self.intersection_emergency_decel = self.get_parameter(
            'intersection_emergency_decel'
        ).value
        self.intersection_stop_line_buffer = self.get_parameter(
            'intersection_stop_line_buffer'
        ).value
        self.intersection_min_stop_duration = self.get_parameter(
            'intersection_min_stop_duration'
        ).value
        self.intersection_proceed_delay = self.get_parameter(
            'intersection_proceed_delay'
        ).value
        self.intersection_clearance_distance = self.get_parameter(
            'intersection_clearance_distance'
        ).value

    def path_callback(self, msg: Path):
        """Callback for path messages."""
        self.current_path = msg
        self.lateral_controller.set_path(msg)
        self.intersection_handler.set_path(msg)

    def odom_callback(self, msg: Odometry):
        """Callback for odometry messages."""
        self.current_odom = msg

    def objects_callback(self, msg: Object3DArray):
        """Callback for detected objects."""
        self.current_objects = msg

    def intersection_callback(self, msg: IntersectionBehavior):
        """Callback for intersection behavior messages."""
        self.intersection_handler.update_intersection_behavior(msg)
        
        self.get_logger().debug(
            f'Intersection behavior: {msg.action}',
            throttle_duration_sec=1.0
        )

    def control_loop(self):
        """Main control loop executed at control_rate Hz."""
        if not self.enabled:
            self._publish_zero_control()
            return

        # Check if we have necessary data
        if self.current_odom is None:
            self.get_logger().warn(
                'No odometry received, publishing zero control',
                throttle_duration_sec=5.0
            )
            self._publish_zero_control()
            return

        if self.current_path is None or len(self.current_path.poses) == 0:
            self.get_logger().warn(
                'No path received, publishing zero control',
                throttle_duration_sec=5.0
            )
            self._publish_zero_control()
            return

        # Compute time step
        current_time = self.get_clock().now()
        dt = (current_time - self.last_control_time).nanoseconds / 1e9
        current_time_sec = current_time.nanoseconds / 1e9
        self.last_control_time = current_time

        # Extract current state
        current_pose = self.current_odom.pose.pose
        current_velocity = self.current_odom.twist.twist.linear
        current_speed = math.sqrt(
            current_velocity.x**2 + current_velocity.y**2
        )

        # Update intersection state
        self.intersection_handler.update_state(
            current_pose,
            current_speed,
            current_time_sec
        )

        # Get intersection-aware target speed
        base_target_speed = self.target_speed
        intersection_target_speed = self.intersection_handler.get_target_speed(
            base_target_speed
        )

        # Lateral control (steering) - unchanged
        steer, cross_track_error = self.lateral_controller.compute_steering(
            current_pose,
            current_speed
        )

        # Longitudinal control (throttle/brake) - intersection-aware
        throttle, brake = self._compute_intersection_aware_control(
            current_speed,
            current_pose,
            intersection_target_speed,
            dt
        )

        # Create and publish control message
        control_msg = VehicleControl()
        control_msg.header.stamp = current_time.to_msg()
        control_msg.header.frame_id = 'base_link'
        control_msg.throttle = float(throttle)
        control_msg.brake = float(brake)
        control_msg.steer = float(steer)
        control_msg.reverse = False

        self.control_pub.publish(control_msg)

        # Publish debug information
        self._publish_debug_info(
            current_speed,
            intersection_target_speed,
            steer,
            throttle,
            brake,
            cross_track_error
        )

        # Publish intersection state
        self._publish_intersection_state()

    def _compute_intersection_aware_control(
        self,
        current_speed: float,
        current_pose: Pose,
        target_speed: float,
        dt: float
    ) -> Tuple[float, float]:
        """
        Compute throttle/brake with intersection awareness.
        
        Args:
            current_speed: Current vehicle speed (m/s).
            current_pose: Current vehicle pose.
            target_speed: Target speed (m/s).
            dt: Time step (s).
        
        Returns:
            Tuple of (throttle, brake).
        """
        intersection_state = self.intersection_handler.state

        # Handle different intersection states
        if intersection_state == IntersectionState.APPROACHING_INTERSECTION:
            # Smooth stop for intersection
            decel = self.intersection_handler.compute_intersection_deceleration(
                current_speed,
                current_pose
            )
            
            if decel is not None:
                # Convert deceleration to brake
                brake = abs(decel / self.max_decel) * self.max_brake
                brake = min(brake, self.max_brake)
                return 0.0, brake
        
        elif intersection_state == IntersectionState.WAITING_AT_INTERSECTION:
            # Hold brake while waiting
            return 0.0, 0.5  # Moderate brake to hold position
        
        elif intersection_state == IntersectionState.CROSSING_INTERSECTION:
            # Cautious crossing - use reduced target speed
            throttle, brake, _ = self.longitudinal_controller.compute_control(
                current_speed,
                self.current_objects,
                dt
            )
            
            # Override target speed for crossing
            self.longitudinal_controller.target_speed = target_speed
            return throttle, brake
        
        elif intersection_state == IntersectionState.EXITING_INTERSECTION:
            # Gradually resume cruise speed
            throttle, brake, _ = self.longitudinal_controller.compute_control(
                current_speed,
                self.current_objects,
                dt
            )
            
            self.longitudinal_controller.target_speed = target_speed
            return throttle, brake
        
        else:
            # Normal cruising
            throttle, brake, _ = self.longitudinal_controller.compute_control(
                current_speed,
                self.current_objects,
                dt
            )
            
            self.longitudinal_controller.target_speed = self.target_speed
            return throttle, brake

    def _publish_zero_control(self):
        """Publish zero control commands (safety fallback)."""
        control_msg = VehicleControl()
        control_msg.header.stamp = self.get_clock().now().to_msg()
        control_msg.header.frame_id = 'base_link'
        control_msg.throttle = 0.0
        control_msg.brake = 0.0
        control_msg.steer = 0.0
        control_msg.reverse = False

        self.control_pub.publish(control_msg)

    def _publish_debug_info(
        self,
        current_speed: float,
        target_speed: float,
        steer: float,
        throttle: float,
        brake: float,
        cross_track_error: float
    ):
        """Publish debug information."""
        lead_info = ""
        if self.longitudinal_controller.lead_vehicle is not None:
            lead_info = (
                f" | Lead: {self.longitudinal_controller.lead_distance:.1f}m"
            )

        intersection_info = f" | Int: {self.intersection_handler.get_state_name()}"

        debug_msg = String()
        debug_msg.data = (
            f"Speed: {current_speed:.2f}/{target_speed:.2f} m/s | "
            f"Steer: {steer:.3f} | "
            f"Throttle: {throttle:.2f} | "
            f"Brake: {brake:.2f} | "
            f"CTE: {cross_track_error:.2f}m"
            f"{lead_info}"
            f"{intersection_info}"
        )

        self.debug_pub.publish(debug_msg)

    def _publish_intersection_state(self):
        """Publish intersection state."""
        state_msg = String()
        state_msg.data = self.intersection_handler.get_state_name()
        self.intersection_state_pub.publish(state_msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    node = AutonomousCruiseIntersectionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
