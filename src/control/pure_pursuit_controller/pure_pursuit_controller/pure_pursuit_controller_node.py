import math

import rclpy
import numpy as np

from numpy import typing as npt

from rclpy.node import Node
from tf_transformations import euler_from_quaternion

from std_msgs.msg import ColorRGBA
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point
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
    MAX_THROTTLE = 0.6
    # Max speed in m/s
    MAX_SPEED: float = 1.5


class VehicleState:
    def __init__(self, l):
        self.l = l
        self.pose: Pose | None = None
        self.velocity: float | None = None

    def loaded(self) -> bool:
        return self.pose is not None and self.velocity is not None

    def calc_throttle_brake(self) -> tuple[float, float] | None:
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
        if not self.loaded():
            return

        dx = target[0] - self.pose.position.x
        dy = target[1] - self.pose.position.y

        _, _, yaw = euler_from_quaternion(
            [
                self.pose.orientation.x,
                self.pose.orientation.y,
                self.pose.orientation.z,
                self.pose.orientation.w,
            ]
        )

        alpha = math.atan2(dy, dx) - yaw
        delta = math.atan2(
            2.0 * Constants.WHEEL_BASE * math.sin(alpha), self.lookahead_distance()
        )

        # Normalize the steering angle from [-pi, pi] to [-1, 1]
        return -delta / (math.pi / 2)

    def lookahead_distance(self) -> float:
        return Constants.kf * self.velocity + Constants.LD

    def _calc_distance(self, x: float, y: float) -> float:
        return math.hypot(
            x - self.pose.position.x,
            y - self.pose.position.y,
        )


class PursuitPath:
    def __init__(self, path: list[tuple[float, float]] = []):
        self.path: npt.NDArray[np.float64] = np.asarray(path)
        self.current_path_range: tuple[int, int] = (0, 0)

    def _calc_nearest_index(self, vs: VehicleState) -> int:
        if len(self.path) == 0 or not vs.loaded():
            return

        min_dist = float("inf")
        min_index = 0
        for i, (x, y) in enumerate(self.path):
            dist = vs._calc_distance(x, y)
            if dist < min_dist:
                min_dist = dist
                min_index = i

        return min_index

    def get_current_path(self) -> npt.NDArray[np.float64]:
        return np.copy(
            self.path[self.current_path_range[0] : self.current_path_range[1] + 1]
        )

    def calc_target_point(self, vs: VehicleState) -> tuple[float, float] | None:
        if len(self.path) == 0 or not vs.loaded():
            return

        if self.current_path_range == (0, 0):
            self.current_path_range = (0, self._calc_nearest_index(vs))

        current_path_end = self.current_path_range[1]
        for i, (x, y) in enumerate(self.path[current_path_end:]):
            dist = vs._calc_distance(x, y)
            if dist > vs.lookahead_distance():
                self.current_path_range = (current_path_end, current_path_end + i)
                return (x, y)


class PursePursuitController(Node):

    def __init__(self):
        super().__init__("pure_pursuit_controler")

        self.vehicle_state = VehicleState(l=self.get_logger())
        self.path = PursuitPath()
        self.target_waypoint = None

        self.first_route = None

        self.route_subscriber = self.create_subscription(
            Path, "/planning/route", self.route_callback, 1
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
        self.rviz_path_publisher = self.create_publisher(Path, "/planning/path", 1)
        self.barrier_marker_pub = self.create_publisher(
            Marker, "/planning/barrier_marker", 1
        )

        self.waypoint_timer = self.create_timer(0.5, self.waypoint_callback)
        self.control_timer = self.create_timer(0.1, self.control_callback)

    def clock_callback(self, msg: Clock):
        self.clock = msg.clock

    def route_callback(self, msg: Path):
        # Only use the first route for now. TODO: how will routes be updated in the future?
        if self.first_route is None:
            self.first_route = msg
            route = [
                (pose_stamped.pose.position.x, pose_stamped.pose.position.y)
                for pose_stamped in msg.poses
            ]
            self.get_logger().info(f"Route: {route}\n")
            self.path.path = np.asarray(route)

    def odometry_callback(self, msg: Odometry):
        self.vehicle_state.pose = msg.pose.pose

    def speed_callback(self, msg: VehicleSpeed):
        self.vehicle_state.velocity = msg.speed

    def waypoint_callback(self):
        self.target_waypoint = self.path.calc_target_point(self.vehicle_state)
        current_path = self.path.get_current_path()

        if len(current_path) == 0:
            return

        vehicle_origin = np.asarray(
            [self.vehicle_state.pose.position.x + 5, self.vehicle_state.pose.position.y]
        )

        # Each RVIZ grid cell is 0.4m.
        relative_waypoint = 0.4 * (np.asarray(self.target_waypoint) - vehicle_origin)

        self.publishLookaheadMarker(
            self.vehicle_state.lookahead_distance(), relative_waypoint
        )

        path_msg = Path()
        path_msg.header.frame_id = "base_link"
        path_msg.header.stamp = self.clock

        vehicle_origin = np.asarray(
            [self.vehicle_state.pose.position.x + 5, self.vehicle_state.pose.position.y]
        )

        current_path[:, 0] -= vehicle_origin[0]
        current_path[:, 1] -= vehicle_origin[1]
        current_path *= 0.4  # Each RVIZ grid cell is 0.4m.

        path_msg.poses = [
            PoseStamped(
                header=path_msg.header,
                pose=Pose(position=Point(x=x - 20, y=y - 30)),
            )
            for x, y in current_path
        ]

        self.get_logger().info(
            f"Current Path: {current_path}. Target: {self.target_waypoint}"
        )

        self.rviz_path_publisher.publish(path_msg)

    def publishLookaheadMarker(self, radius: float, pose: tuple[float, float]):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.clock
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.CYLINDER

        marker.action = Marker.ADD

        marker.scale.x = radius * 2
        marker.scale.y = radius * 2
        marker.scale.z = 0.2

        color = ColorRGBA()
        color.a = 0.3
        color.g = 1.0
        color.b = 1.0
        marker.color = color

        self.barrier_marker_pub.publish(marker)

        marker.id = 1
        marker.type = Marker.ARROW

        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 0.8
        marker.scale.z = 0.3

        color = ColorRGBA()
        color.a = 0.7
        color.g = 1.0
        color.b = 0.6
        marker.color = color

        pt_a = Point()
        marker.points.append(pt_a)

        pt_b = Point()
        pt_b.x = pose[0]
        pt_b.y = pose[1]
        marker.points.append(pt_b)

        self.barrier_marker_pub.publish(marker)

    def control_callback(self):
        # If no target waypoint, do nothing.
        if not self.target_waypoint:
            return

        # Calculate throttle, brake, and steer
        throttle_brake = self.vehicle_state.calc_throttle_brake()
        steer = self.vehicle_state.calc_steer(self.target_waypoint)

        # If vehicle state is not loaded, do nothing.
        if not throttle_brake or not steer:
            return

        # Set throttle, brake, and steer and publish.
        throttle, brake = throttle_brake
        control_msg = VehicleControl()
        control_msg.throttle = throttle
        control_msg.brake = brake
        control_msg.steer = steer
        self.get_logger().info(
            f"Steer: {control_msg.steer} Throttle: {throttle} Brake: {brake}"
        )
        self.command_publisher.publish(control_msg)


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
