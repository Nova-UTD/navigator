#!/usr/bin/env python3
"""
Main autonomous cruise controller node (basic version).

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
from sensor_msgs.msg import PointCloud2
from navigator_msgs.msg import VehicleControl, Object3DArray, VehicleSpeed, IntersectionBehavior
from std_msgs.msg import String
from geometry_msgs.msg import Pose

import math
import numpy as np
from typing import Optional, Tuple

from autonomous_cruise.lateral_controller import PurePursuitController
from autonomous_cruise.longitudinal_controller import AdaptiveLongitudinalController

# The LiDAR is roof-mounted with a steep downward FOV, so returns closer than
# this are the vehicle's own hood/bumper, not real obstacles, and must be
# filtered out. STOP_DIST (below) must stay >= this value with margin — if it
# doesn't, a closing obstacle's points get filtered out (lidar_obstacle_distance
# resets to inf) before the vehicle ever reaches STOP_DIST, so it accelerates
# back to cruise speed right as it's about to hit something.
LIDAR_MIN_RANGE_M = 2.5


class AutonomousCruiseController(Node):
    """
    Autonomous cruise controller node combining lateral and longitudinal control.

    This node implements a complete autonomous driving system for highway
    cruising, including:
    - Lateral control (steering) using Pure Pursuit algorithm
    - Longitudinal control (speed) using adaptive PID
    - Adaptive cruise control with safe vehicle following
    - Integration with Navigator perception and planning stack
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

        # State variables
        self.current_path: Optional[Path] = None
        self.current_odom: Optional[Odometry] = None
        self.current_objects: Optional[Object3DArray] = None
        self.current_speed: float = 0.0
        self.intersection_action: str = 'Proceed'  # 'Wait' = stop, 'Proceed' = go
        self.traffic_light_red: bool = False
        self.lidar_obstacle_distance: float = float('inf')  # m to nearest forward obstacle
        # Hysteresis latch: once fully stopped for an obstacle, stay stopped
        # until it's clearly gone (past SLOW_DIST), not just a hair past
        # STOP_DIST. Without this, sensor noise / minor position drift right
        # at the STOP_DIST boundary let the vehicle creep forward again
        # immediately after stopping, inching into the obstacle over repeated
        # stop/creep cycles.
        self._stopped_for_obstacle = False
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

        self.speed_sub = self.create_subscription(
            VehicleSpeed,
            '/speed',
            self.speed_callback,
            qos_best_effort
        )

        self.intersection_sub = self.create_subscription(
            IntersectionBehavior,
            '/intersection',
            self.intersection_callback,
            qos_reliable
        )
        self.traffic_light_sub = self.create_subscription(
            String,
            "/carla/traffic_light_state",
            self.traffic_light_callback,
            qos_reliable
        )

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar/filtered',
            self.lidar_callback,
            qos_best_effort
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

        # Control loop timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )

        self.get_logger().info('Autonomous Cruise Controller initialized')
        self.get_logger().info(f'Target speed: {self.target_speed:.2f} m/s')
        self.get_logger().info(f'Control rate: {self.control_rate} Hz')
        self.get_logger().info(f'Subscribing to path: {self.path_topic}')
        self.get_logger().info(f'Subscribing to odometry: {self.odom_topic}')
        self.get_logger().info(f'Subscribing to objects: {self.objects_topic}')
        self.get_logger().info(f'Publishing control: {self.control_topic}')

    def _declare_parameters(self):
        """Declare ROS2 parameters."""
        # Topic names
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('odom_topic', '/gnss/odometry')
        self.declare_parameter('objects_topic', '/tracked/objects3d')
        self.declare_parameter('control_topic', '/vehicle/control')

        # Control parameters
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('enabled', True)

        # Vehicle parameters
        self.declare_parameter('wheelbase', 2.875)

        # Lateral control parameters
        self.declare_parameter('min_lookahead', 3.0)
        self.declare_parameter('max_lookahead', 15.0)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('max_steer', 1.0)

        # Longitudinal control parameters
        self.declare_parameter('target_speed', 6.0)
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.15)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('max_accel', 2.5)
        self.declare_parameter('max_decel', -3.5)
        self.declare_parameter('max_jerk', 3.0)
        self.declare_parameter('max_throttle', 0.75)
        self.declare_parameter('max_brake', 1.0)
        self.declare_parameter('integral_limit', 5.0)

        # Adaptive cruise parameters
        self.declare_parameter('time_gap', 1.5)
        self.declare_parameter('min_following_distance', 5.0)
        self.declare_parameter('max_detection_distance', 50.0)
        self.declare_parameter('detection_angle', 0.3)

    def _load_parameters(self):
        """Load parameters from ROS2 parameter server."""
        # Topic names
        self.path_topic = self.get_parameter('path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.objects_topic = self.get_parameter('objects_topic').value
        self.control_topic = self.get_parameter('control_topic').value

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

    def path_callback(self, msg: Path):
        """
        Callback for path messages.

        Args:
            msg: Path message from path planner.
        """
        self.current_path = msg
        self.lateral_controller.set_path(msg)

        self.get_logger().debug(
            f'Received path with {len(msg.poses)} waypoints'
        )

    def odom_callback(self, msg: Odometry):
        """
        Callback for odometry messages.

        Args:
            msg: Odometry message with vehicle state.
        """
        self.current_odom = msg

    def objects_callback(self, msg: Object3DArray):
        """
        Callback for detected objects.

        Args:
            msg: Array of detected objects from perception.
        """
        self.current_objects = msg

        self.get_logger().debug(
            f'Received {len(msg.objects)} detected objects'
        )

    def speed_callback(self, msg: VehicleSpeed):
        """Callback for vehicle speed from GNSS processor."""
        self.current_speed = msg.speed

    def intersection_callback(self, msg: IntersectionBehavior):
        """Callback for intersection manager commands (Wait / Proceed)."""
        self.intersection_action = msg.action

    def traffic_light_callback(self, msg: String):
        self.traffic_light_red = (msg.data == "Red")

    def lidar_callback(self, msg: PointCloud2):
        """Scan ground-segmented LiDAR for obstacles on the planned path only."""
        if msg.width == 0 or msg.point_step == 0:
            return
        step = msg.point_step // 4
        raw = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if len(raw) < step:
            return
        xs = raw[0::step]
        ys = raw[1::step]
        zs = raw[2::step]

        # Pre-filter: ahead of bumper, not too far, above ground
        pre = (xs > LIDAR_MIN_RANGE_M) & (xs < 20.0) & (zs > 0.3)
        if not pre.any():
            self.lidar_obstacle_distance = float('inf')
            return
        xs_f, ys_f = xs[pre], ys[pre]

        path = self.current_path
        if path is not None and len(path.poses) > 1:
            wx = np.array([p.pose.position.x for p in path.poses])
            wy = np.array([p.pose.position.y for p in path.poses])
            ahead = (wx > 1.0) & (wx < 20.0)
            if ahead.any():
                wx, wy = wx[ahead], wy[ahead]
                dx = xs_f[:, None] - wx[None, :]
                dy = ys_f[:, None] - wy[None, :]
                on_path = np.sqrt(dx**2 + dy**2).min(axis=1) < 0.6
                self.lidar_obstacle_distance = float(xs_f[on_path].min()) if on_path.sum() >= 5 else float('inf')
                return

        # Fallback: tight rectangular corridor when no path available
        mask = (np.abs(ys_f) < 0.9)
        self.lidar_obstacle_distance = float(xs_f[mask].min()) if mask.sum() >= 5 else float('inf')

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
        self.last_control_time = current_time

        # Extract current state
        current_pose = self.current_odom.pose.pose
        current_speed = self.current_speed

        # Lateral control (steering)
        steer, cross_track_error = self.lateral_controller.compute_steering(
            current_pose,
            current_speed
        )

        # LiDAR-based speed limit — scale down as obstacle approaches.
        # STOP_DIST must stay above LIDAR_MIN_RANGE_M (with margin) — see
        # comment there. Otherwise the obstacle becomes invisible to this
        # check before the vehicle has actually stopped.
        # Doubled from the original 7.0/STOP_DIST+0.5m margin: the logic
        # threshold being correct doesn't help if there isn't enough
        # physical distance left to actually decelerate to zero in time.
        SLOW_DIST = 14.0   # m — begin decelerating
        STOP_DIST = LIDAR_MIN_RANGE_M + 3.5   # m — full stop
        CREEP = 1.0       # m/s — minimum speed while obstacle present (lets planner replan)
        d = self.lidar_obstacle_distance
        if d < STOP_DIST:
            self.longitudinal_controller.target_speed = 0.0
            self._stopped_for_obstacle = True
        elif d < SLOW_DIST:
            if self._stopped_for_obstacle:
                # Latched: stay stopped until the obstacle is clearly gone,
                # not just a hair past STOP_DIST. See hysteresis comment above.
                self.longitudinal_controller.target_speed = 0.0
            else:
                ratio = (d - STOP_DIST) / (SLOW_DIST - STOP_DIST)
                self.longitudinal_controller.target_speed = max(CREEP, self.target_speed * ratio)
        else:
            self.longitudinal_controller.target_speed = self.target_speed
            self._stopped_for_obstacle = False

        # Longitudinal control (throttle/brake)
        throttle, brake, target_speed = (
            self.longitudinal_controller.compute_control(
                current_speed,
                self.current_objects,
                dt
            )
        )

        # Intersection / traffic light override — stop on red
        if self.intersection_action == 'Wait' or self.traffic_light_red:
            throttle = 0.0
            brake = 1.0

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
            target_speed,
            steer,
            throttle,
            brake,
            cross_track_error
        )

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
        """
        Publish debug information.

        Args:
            current_speed: Current vehicle speed (m/s).
            target_speed: Target speed (m/s).
            steer: Steering command.
            throttle: Throttle command.
            brake: Brake command.
            cross_track_error: Distance from path (m).
        """
        lead_info = ""
        if self.longitudinal_controller.lead_vehicle is not None:
            lead_info = (
                f" | Lead: {self.longitudinal_controller.lead_distance:.1f}m"
            )

        debug_msg = String()
        debug_msg.data = (
            f"Speed: {current_speed:.2f}/{target_speed:.2f} m/s | "
            f"Steer: {steer:.3f} | "
            f"Throttle: {throttle:.2f} | "
            f"Brake: {brake:.2f} | "
            f"CTE: {cross_track_error:.2f}m"
            f"{lead_info}"
        )

        self.debug_pub.publish(debug_msg)


def main(args=None):
    """Main entry point for the node."""
    rclpy.init(args=args)

    node = AutonomousCruiseController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
