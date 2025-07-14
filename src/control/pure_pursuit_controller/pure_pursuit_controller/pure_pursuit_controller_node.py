"""
ROS2 Node for the Pure Pursuit Controller. 

NOT WORKING as of 04/30/2024, mainly due to faulty path planners (RTP, Nav2, attempts at Graph/A* path planning).
"""

import math

import rclpy
import numpy as np

from numpy import typing as npt

from rclpy.node import Node

from std_msgs.msg import ColorRGBA, Header
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point, Vector3
from visualization_msgs.msg import Marker

from navigator_msgs.msg import VehicleControl, VehicleSpeed


class Constants:
    # Look ahead distance in meters
    LD: float = 3.0
    # Look forward gain in meters (gain in look ahead distance per m/s of speed)
    kf: float = 0.1
    # Wheel base (distance between front and rear wheels) in meter
    WHEEL_BASE: float = 3.5
    # Max throttle and acceleration
    MAX_THROTTLE = 2
    # Max speed in m/s
    MAX_SPEED: float = 1.5
    # Stopping distance in meters
    # This is the distance from the end of the path at which the vehicle will start to brake.
    STOPPING_DIST: float = 10
    # Max break possible
    MAX_BREAK: float = 1.0
    # Max steering angle in radians.
    MAX_STEERING_ANGLE = 0.58


class VehicleState:
    """Vehicle state class that holds the current pose and velocity of the vehicle."""

    def __init__(self):
        self.pose: Pose | None = None
        self.velocity: float | None = None

    def loaded(self) -> bool:
        """Check if the vehicle state is loaded with pose and velocity.

        Returns:
            bool: True if pose and velocity are not None, False otherwise.
        """
        return self.pose is not None and self.velocity is not None

    def calc_throttle_brake(self) -> tuple[float, float] | None:
        """Calculate throttle and brake based on the current velocity.

        Returns:
            tuple[float, float] | None: Throttle and brake values (None if vehicle state is not fulled loaded).
        """
        if not self.loaded():
            return

        # Our target speed is the max speed
        pid_error = Constants.MAX_SPEED - self.velocity
        if pid_error > 0:
            throttle = min(pid_error * 0.5, Constants.MAX_THROTTLE)
            brake = 0.0
        elif pid_error <= 0:
            brake = pid_error * Constants.MAX_THROTTLE * -1.0
            throttle = 0.0

        return throttle, brake

    def calc_steer(self, target: tuple[float, float]) -> float | None:
        """Calculate the steering angle based on the target point.

        Args:
            target (tuple[float, float]): Target point (x, y) to steer towards.

        Returns:
            float | None: Steering angle in the range [-1, 1] (None if vehicle state is not fulled loaded).
        """
        if not self.loaded():
            return

        alpha = math.atan2(target[1], target[0])

        # delta = (
        #     2.0 * Constants.WHEEL_BASE * math.sin(alpha) / math.hypot(target[0], target[1])
        # )

        # Alpha works with RPT node to a limited extent -- only in parking lot, 

        # Originally, delta was used (and working in Carla when following smooth route),
        # but simply did not work in real world testing.
        # Delta is derived following this article: https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html.

        # We clip and normalize alpha to a max steering angle, which we believe is ~0.58 as specified by the Unified Controller.
        clipped_steering_angle = self.steering_angle_to_wheel(-alpha)
        normalized = clipped_steering_angle / Constants.MAX_STEERING_ANGLE

        return normalized


    def steering_angle_to_wheel(self, angle: float) -> float:
        """
        Converts a steering angle to a steering position
        """
        return float(min(max(angle, -Constants.MAX_STEERING_ANGLE), Constants.MAX_STEERING_ANGLE))


    def lookahead_distance(self) -> float:
        """Calculate the lookahead distance based on the current velocity.

        Returns:
            float: Lookahead distance in meters.
        """
        return Constants.kf * self.velocity + Constants.LD


class PursuitPath:
    """Pursuit path class that holds the path and calculates the target point for the vehicle.

    Args:
        path (list[tuple[float, float]], optional): List of points (x, y) that represent the path. Defaults to [].
    """

    def __init__(self, path: list[tuple[float, float]] = []):
        self.path: npt.NDArray[np.float64] = np.asarray(path)
        self.target_index: int | None = None
    def distance(self) -> float:
        if len(self.path) == 0:
            return 0.0

        x, y = self.path[-1]
        return math.hypot(x, y)

    def set_path(self, path: list[tuple[float, float]]) -> None:
        """Set the path.

        Args:
            path (list[tuple[float, float]]): List of points (x, y) that represent the path.
        """
        self.path = np.asarray(path)
        self.target_index = None

    def _calc_nearest_index(self) -> int:
        """Calculate the nearest index of the path to the vehicle's current position.

        Returns:
            int: Nearest index of the path to the vehicle's current position.
        """
        min_dist = float("inf")
        min_index = 0
        for i, (x, y) in enumerate(self.path):
            dist = math.hypot(x, y)
            if dist < min_dist:
                min_dist = dist
                min_index = i

        return min_index

    def get_current_path(self) -> npt.NDArray[np.float64]:
        """Get the current path up to and including the target point."""
        target_index = self.target_index + 1 if self.target_index is not None else 0
        return np.copy(self.path[:target_index])

    def calc_target_point(self, vs: VehicleState) -> tuple[float, float] | None:
        """Calculate the target point based on the vehicle state.

        Args:
            vs (VehicleState): Vehicle state object that holds the current pose and velocity of the vehicle.

        Returns:
            tuple[float, float] | None: Target point (x, y) to steer towards (None if path is empty or vehicle state is not fulled loaded).
        """
        if len(self.path) == 0 or not vs.loaded():
            return

        # If calculating the target point for the first time, find the nearest index to the vehicle's current position.
        if self.target_index is None:
            self.target_index = self._calc_nearest_index()

        # When the path is shorter then the lookahead, we will pick the last point.
        if self.distance() < vs.lookahead_distance():
            x, y = self.path[-1]
            return (x, y)

        # Find the first point that is further than the lookahead distance.
        # If looking at the same path, start from the last target index.
        for i, (x, y) in enumerate(self.path[self.target_index :]):
            dist = math.hypot(x, y)
            if dist > vs.lookahead_distance():
                self.target_index = i
                return (x, y)


class PursePursuitController(Node):
    """Pure pursuit controller node that publishes vehicle control commands based on the pure pursuit algorithm."""

    def __init__(self):
        super().__init__("pure_pursuit_controler")

        self.vehicle_state = VehicleState()
        self.path = PursuitPath()
        self.target_waypoint = None
        self.clock = Clock().clock

        self.route_subscriber = self.create_subscription(
            Path, "/planning/path", self.route_callback, 1
        )
        self.odometry_subscriber = self.create_subscription(
            Odometry, "/gnss/odometry", self.odometry_callback, 1
        )
        self.speed_subscriber = self.create_subscription(
            VehicleSpeed, "/speed", self.speed_callback, 1
        )
        self.clock_subscriber = self.create_subscription(
            Clock, "/clock", self.clock_callback, 1
        )

        self.command_publisher = self.create_publisher(
            VehicleControl, "/vehicle/control", 1
        )
        self.barrier_marker_pub = self.create_publisher(
            Marker, "/planning/barrier_marker", 1
        )
        self.lookahead_path_publisher = self.create_publisher(Path, "/ppc/path", 1)

        self.control_timer = self.create_timer(0.05, self.control_callback)
        self.visualize_path_timer = self.create_timer(0.1, self.visualize_path_callback)
        self.visualize_waypoint_timer = self.create_timer(
            0.1, self.visualize_waypoint_callback
        )

    def clock_callback(self, msg: Clock):
        self.clock = msg.clock

    def odometry_callback(self, msg: Odometry):
        self.vehicle_state.pose = msg.pose.pose

    def speed_callback(self, msg: VehicleSpeed):
        self.vehicle_state.velocity = msg.speed

    def route_callback(self, msg: Path):
        """Callback function for the path subscriber.

        Recalculates the path and target waypoint based on the received path message.
        """
        path = [
            (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
            for pose_stamped in msg.poses
        ]
        self.path.set_path(path)

    def stop_vehicle(self, steer: float, break_value: float):
        """Stop the vehicle by applying the break."""
        control_msg = VehicleControl()
        control_msg.brake = break_value
        control_msg.steer = steer
        self.command_publisher.publish(control_msg)

    def control_callback(self):
        """Calculate and publish the vehicle control commands based on the pure pursuit algorithm."""
        # Calculate the waypoint just outside the lookahead distance.
        self.target_waypoint = self.path.calc_target_point(self.vehicle_state)

        # If no target waypoint, do nothing.
        if not self.target_waypoint:
            return

        # ------------------------
        # Stopping distance and easing stop was not tested
        # because base pure pursuit functinality was not working to begin with.
        # Uncomment in future if/when pure pursuit integrates with rest of the stack.
        #
        #
        # path_dist = self.path.distance()
        # if path_dist < Constants.STOPPING_DIST:
        #     # Scale the break value linearly based on the distance to the end of the path.
        #     # Break scale is 1.0 at the end of the path and 0.0 at the stopping distance.
        #     break_scale = 1.0 - (path_dist / Constants.STOPPING_DIST)
        #     break_value = min(
        #         Constants.MAX_BREAK,
        #         break_scale * Constants.MAX_BREAK,
        #     )
        #     steer = self.vehicle_state.calc_steer(self.target_waypoint)
        #     self.stop_vehicle(steer, break_value)
        #     return
        #
        #
        # ------------------------

        # Calculate throttle, brake, and steer
        throttle_brake = self.vehicle_state.calc_throttle_brake()
        steer = self.vehicle_state.calc_steer(self.target_waypoint)

        # If vehicle state is not loaded, do nothing.
        if throttle_brake is None or steer is None:
            return

        # Set throttle, brake, and steer and publish.
        throttle, brake = throttle_brake
        control_msg = VehicleControl()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steer = steer
        self.command_publisher.publish(control_msg)

    def visualize_waypoint_callback(self):
        """Visualize the target waypoint in RVIZ."""
        if self.target_waypoint is None:
            return

        relative_waypoint = 0.4 * np.asarray(self.target_waypoint)

        lookahead_dist = self.vehicle_state.lookahead_distance()
        radius_marker = Marker(
            header=Header(frame_id="base_link", stamp=self.clock),
            ns="lookahead",
            id=0,
            type=Marker.CYLINDER,
            action=Marker.ADD,
            scale=Vector3(x=lookahead_dist * 2, y=lookahead_dist * 2, z=0.2),
            color=ColorRGBA(a=0.3, g=1.0, b=1.0),
        )
        self.barrier_marker_pub.publish(radius_marker)

        arrow_marker = Marker(
            header=Header(frame_id="base_link", stamp=self.clock),
            ns="lookahead",
            id=1,
            type=Marker.ARROW,
            action=Marker.ADD,
            scale=Vector3(x=0.5, y=0.8, z=0.3),
            color=ColorRGBA(a=0.7, g=1.0, b=0.6),
            points=[Point(), Point(x=relative_waypoint[0], y=relative_waypoint[1])],
        )

        self.barrier_marker_pub.publish(arrow_marker)

    def visualize_path_callback(self):
        """Visualize the path in RVIZ."""
        current_path = self.path.get_current_path()

        if len(current_path) == 0:
            return

        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.clock

        # Each RVIZ grid cell is 0.4m.
        path = np.copy(np.asarray(current_path)) * 0.4

        path_msg.poses = [
            PoseStamped(
                header=path_msg.header,
                pose=Pose(position=Point(x=x, y=y)),
            )
            for x, y in path
        ]

        self.lookahead_path_publisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PursePursuitController()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
