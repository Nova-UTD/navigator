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
            max_steer=self.max_steer,
            rear_axle_offset=self.rear_axle_offset
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
        self.lidar_obstacle_distance: float = float('inf')  # m from front bumper to nearest obstacle in our way
        self.route_local: Optional[np.ndarray] = None  # (N, 2) route points around the car, base_link
        self.obstacle_distance: Optional[float] = None  # last obstacle distance, dead-reckoned through dropouts
        self.obstacle_seen_time: float = 0.0
        self.stopped_for_obstacle = False
        self.obstacle_clear_since: Optional[float] = None
        self.obstacle_brake = 0.0
        self.lidar_hits = 0  # consecutive LiDAR frames with an obstacle in the way
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

        # Route around the car (base_link), used to follow the lane past the end of the path
        self.route_local_sub = self.create_subscription(
            Path,
            '/planning/route_local',
            self.route_local_callback,
            1
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
        self.declare_parameter('rear_axle_offset', 1.4)  # m behind base_link (vehicle centre)

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

        # LiDAR braking: distance from the LiDAR (vehicle centre) to the front bumper
        self.declare_parameter('front_bumper_offset', 2.4)

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
        self.rear_axle_offset = self.get_parameter('rear_axle_offset').value

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
        self.front_bumper_offset = self.get_parameter('front_bumper_offset').value

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

    def route_local_callback(self, msg: Path):
        if len(msg.poses) < 2:
            self.route_local = None
            return
        self.route_local = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])

    def _path_continuation(self, end_x: float, end_y: float, heading: float, length: float) -> np.ndarray:
        """Points continuing `length` metres past the end of the path.

        The planner ends the path just before anything it treats as an obstacle (e.g. a stopped
        car), so the obstacle itself is past the path's end. Follow the route from there, since it
        keeps following the lane through turns; fall back to the path's last heading without it.
        """
        route = self.route_local
        if route is not None:
            k = int(np.argmin(np.hypot(route[:, 0] - end_x, route[:, 1] - end_y)))
            if np.hypot(route[k, 0] - end_x, route[k, 1] - end_y) < 2.0:
                ahead = route[k:]
                seg = np.hypot(np.diff(ahead[:, 0]), np.diff(ahead[:, 1]))
                travelled = np.concatenate([[0.0], np.cumsum(seg)])
                return ahead[travelled <= length]
        ext = np.arange(0.5, length + 0.01, 0.5)
        return np.stack([end_x + ext * np.cos(heading), end_y + ext * np.sin(heading)], axis=1)

    def lidar_callback(self, msg: PointCloud2):
        """Scan ground-segmented LiDAR for obstacles in the way of the planned path.

        Only points within PATH_HALF_WIDTH of the path, or of its continuation along the route
        just past its end, count; anything else the LiDAR sees (parked cars, buildings on a turn)
        is ignored. The distance is measured along the path from the front bumper, so on a turn an
        obstacle around the corner is as far away as the drive to it, not its straight-line x.
        """
        LOOK_AHEAD = 20.0      # m from the LiDAR
        PATH_HALF_WIDTH = 1.0  # m — ego half-width plus a little margin
        CONTINUATION = 6.0     # m — the planner stops the path ~1-2 m short of an obstacle
        MIN_POINTS = 5
        CONFIRM_FRAMES = 2     # a one-frame blip (curb, pole at a corner) must not slam the brakes

        if msg.width == 0 or msg.point_step == 0:
            return
        step = msg.point_step // 4
        raw = np.frombuffer(bytes(msg.data), dtype=np.float32)
        if len(raw) < step:
            return
        xs = raw[0::step]
        ys = raw[1::step]
        zs = raw[2::step]

        # Pre-filter: ahead of the LiDAR, not too far, above ground. 0.5 m excludes ego returns.
        pre = (xs > 0.5) & (xs < LOOK_AHEAD) & (zs > 0.3)
        if not pre.any():
            self.lidar_hits = 0
            self.lidar_obstacle_distance = float('inf')
            return
        xs_f, ys_f = xs[pre], ys[pre]

        path = self.current_path
        if path is not None and len(path.poses) > 1:
            wx = np.array([p.pose.position.x for p in path.poses])
            wy = np.array([p.pose.position.y for p in path.poses])
            ahead = (wx > 0.0) & (wx < LOOK_AHEAD)
            wx, wy = wx[ahead], wy[ahead]
            if len(wx) == 0:
                wx, wy = np.array([0.0]), np.array([0.0])
            heading = np.arctan2(wy[-1] - wy[-2], wx[-1] - wx[-2]) if len(wx) >= 2 else 0.0
            cont = self._path_continuation(wx[-1], wy[-1], heading, CONTINUATION)
            wx = np.concatenate([[0.0], wx, cont[:, 0]])
            wy = np.concatenate([[0.0], wy, cont[:, 1]])
            arc = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(wx), np.diff(wy)))])
            dx = xs_f[:, None] - wx[None, :]
            dy = ys_f[:, None] - wy[None, :]
            dist = np.sqrt(dx**2 + dy**2)
            in_way = dist.min(axis=1) < PATH_HALF_WIDTH
            along = arc[dist.argmin(axis=1)]  # distance along the path to each point
        else:
            # No path: straight corridor
            in_way = np.abs(ys_f) < PATH_HALF_WIDTH
            along = xs_f

        if in_way.sum() >= MIN_POINTS:
            self.lidar_hits += 1
        else:
            self.lidar_hits = 0
        if self.lidar_hits >= CONFIRM_FRAMES:
            self.lidar_obstacle_distance = max(float(along[in_way].min()) - self.front_bumper_offset, 0.0)
        else:
            self.lidar_obstacle_distance = float('inf')

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

        # LiDAR-based speed limit: ease down to a stop STOP_GAP short of an obstacle in our way.
        # Distances are from the front bumper (see lidar_callback).
        STOP_GAP = 4.0         # m — where we come to rest behind the obstacle
        COMFORT_DECEL = 1.0    # m/s^2 — deceleration the approach is planned around
        FULL_BRAKE_DECEL = 6.0  # m/s^2 — rough deceleration at brake = 1.0, maps decel to brake
        BRAKE_RATE = 0.8       # brake units per second — how fast the brake pedal may move
        URGENT_DECEL = 2.5     # m/s^2 — above this, brake at once instead of easing in
        MEMORY = 0.6           # s — bridge a couple of dropped LiDAR frames (~3-4 Hz), no longer
        FRESH = 0.5            # s — only a detection this recent can latch a stop
        APPROACH_THROTTLE = 0.3  # max throttle while an obstacle is limiting our speed
        RELEASE_GAP = STOP_GAP + 2.0  # m — obstacle must be this far (or gone) to drive off again
        RELEASE_TIME = 1.5     # s — ...for this long
        now_s = current_time.nanoseconds / 1e9

        # Obstacle distance (along the path), dead-reckoned through a dropped frame or two so a
        # single miss does not read as "road clear" and make the car lurch forward.
        if np.isfinite(self.lidar_obstacle_distance):
            self.obstacle_distance = self.lidar_obstacle_distance
            self.obstacle_seen_time = now_s
        elif self.obstacle_distance is not None and now_s - self.obstacle_seen_time < MEMORY:
            self.obstacle_distance = max(self.obstacle_distance - current_speed * dt, 0.0)
        else:
            self.obstacle_distance = None
        d = self.obstacle_distance if self.obstacle_distance is not None else float('inf')

        # Once stopped for an obstacle we actually still see, stay stopped until it has cleared.
        fresh = np.isfinite(d) and now_s - self.obstacle_seen_time < FRESH
        if fresh and d <= STOP_GAP + 0.5 and current_speed < 0.3:
            self.stopped_for_obstacle = True
        if self.stopped_for_obstacle:
            if d > RELEASE_GAP:
                if self.obstacle_clear_since is None:
                    self.obstacle_clear_since = now_s
                elif now_s - self.obstacle_clear_since > RELEASE_TIME:
                    self.stopped_for_obstacle = False
            else:
                self.obstacle_clear_since = None

        # Fastest speed from which COMFORT_DECEL still stops us at STOP_GAP.
        allowed_speed = np.sqrt(2.0 * COMFORT_DECEL * max(d - STOP_GAP, 0.0)) if np.isfinite(d) else float('inf')
        if self.stopped_for_obstacle:
            self.longitudinal_controller.target_speed = 0.0
        else:
            self.longitudinal_controller.target_speed = min(self.target_speed, allowed_speed)

        # Longitudinal control (throttle/brake)
        throttle, brake, target_speed = (
            self.longitudinal_controller.compute_control(
                current_speed,
                self.current_objects,
                dt
            )
        )

        # Brake for the obstacle directly: the deceleration needed to stop at STOP_GAP, with the
        # pedal moving at most BRAKE_RATE per second so the stop is smooth rather than stop-start.
        urgent = False
        if self.stopped_for_obstacle or d <= STOP_GAP:
            wanted_brake = self.max_brake if current_speed < 0.3 else min(
                self.max_brake, COMFORT_DECEL * 2.0 / FULL_BRAKE_DECEL + 0.2)
        elif np.isfinite(d) and current_speed > allowed_speed:
            needed_decel = current_speed ** 2 / (2.0 * max(d - STOP_GAP, 0.3))
            wanted_brake = min(self.max_brake, needed_decel / FULL_BRAKE_DECEL)
            urgent = needed_decel > URGENT_DECEL  # something appeared close: no time to ease in
        else:
            wanted_brake = 0.0
        if urgent and wanted_brake > self.obstacle_brake:
            self.obstacle_brake = wanted_brake
        else:
            step_limit = BRAKE_RATE * dt
            self.obstacle_brake += float(np.clip(wanted_brake - self.obstacle_brake, -step_limit, step_limit))
        if self.obstacle_brake > 0.01:
            throttle = 0.0
            brake = max(brake, self.obstacle_brake)

        # While an obstacle is limiting our speed, don't let the PID surge past the allowed speed
        # (full throttle up, brake, stop, repeat): coast near the limit, gentle throttle below it,
        # and drop any integral wind-up that would push us forward.
        if self.stopped_for_obstacle or allowed_speed < self.target_speed:
            self.longitudinal_controller.integral = min(self.longitudinal_controller.integral, 0.0)
            if self.stopped_for_obstacle or current_speed >= allowed_speed - 0.2:
                throttle = 0.0
            else:
                throttle = min(throttle, APPROACH_THROTTLE)

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
