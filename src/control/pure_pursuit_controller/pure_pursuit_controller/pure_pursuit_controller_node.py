import math

import rclpy

from rclpy.node import Node
from tf_transformer import euler_from_quaternion

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Pose, PoseStamped, Point

from navigator_msgs.msg import VehicleControl, VehicleSpeed


class Constants:
    # Look ahead distance in meters
    LD: float = 6.0
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
        return -delta / math.pi

    def lookahead_distance(self) -> float:
        return Constants.kf * self.velocity + Constants.LD

    def _calc_distance(self, x: float, y: float) -> float:
        return math.hypot(
            x - self.pose.position.x,
            y - self.pose.position.y,
        )


class PursuitPath:
    def __init__(self, cx: list[float] = [], cy: list[float] = []):
        self.cx = cx
        self.cy = cy
        self.prev_index: int | None = None

    def _calc_nearest_index(self, vs: VehicleState) -> int:
        if len(self.cx) == 0 or len(self.cy) == 0 or not vs.loaded():
            return

        min_dist = float("inf")
        min_index = 0
        for i in range(len(self.cx)):
            dist = vs._calc_distance(self.cx[i], self.cy[i])
            if dist < min_dist:
                min_dist = dist
                min_index = i

        return min_index

    def calc_target_point(self, vs: VehicleState) -> tuple[float, float] | None:
        if len(self.cx) == 0 or len(self.cy) == 0 or not vs.loaded():
            return

        if self.prev_index is None:
            self.prev_index = self._calc_nearest_index(vs)

        for i in range(self.prev_index, len(self.cx)):
            dist = vs._calc_distance(self.cx[i], self.cy[i])
            if dist > vs.lookahead_distance():
                self.prev_index = i
                return [self.cx[i], self.cy[i]]


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

        self.command_publisher = self.create_publisher(
            VehicleControl, "/vehicle/control", 1
        )

        self.waypoint_timer = self.create_timer(0.5, self.waypoint_callback)
        self.control_timer = self.create_timer(0.1, self.control_callback)

    def route_callback(self, msg: Path):
        # Only use the first route for now. TODO: how will routes be updated in the future?
        if self.first_route is None:
            self.first_route = msg
            route = [
                [pose_stamped.pose.position.x, pose_stamped.pose.position.y]
                for pose_stamped in msg.poses
            ]
            self.get_logger().info(f"Route: {route}\n")
            self.path.cx = [pose_stamped.pose.position.x for pose_stamped in msg.poses]
            self.path.cy = [pose_stamped.pose.position.y for pose_stamped in msg.poses]

    def odometry_callback(self, msg: Odometry):
        self.vehicle_state.pose = msg.pose.pose

    def speed_callback(self, msg: VehicleSpeed):
        self.vehicle_state.velocity = msg.speed

    def waypoint_callback(self):
        self.target_waypoint = self.path.calc_target_point(self.vehicle_state)

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
