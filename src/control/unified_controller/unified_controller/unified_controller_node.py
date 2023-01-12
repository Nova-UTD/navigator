#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2022 - https://github.com/Nova-UTD/navigator/
# Will Heitman - Will.Heitman@utdallas.edu

from typing import List
from nova_msgs.msg import Trajectory, TrajectoryPoint, PeddlePosition, SteeringPosition
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion, Vector3
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import collections

class UnifiedController(Node):

    MAX_SAFE_DECELERATION = 8 # m/s^2
    MAX_COMFORTABLE_DECELERATION = 1 # m/s^2
    MAX_COMFORTABLE_ACCELERATION = 1 
    MIN_TIME_LOOKAHEAD = 10

    STEERING_LOOKEAHAD_DISTANCE = 25 # meters

    MAX_STEERING_ANGLE = 0.58294 # radians

    cached_odometry: Odometry = Odometry()
    cached_path: Trajectory = Trajectory()

    def paths_cb(self, msg: Trajectory):
        self.cached_path = msg

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg

    def generate_commands(self):

        if len(self.cached_path.points) == 0:
            return

        # Current vehicle state
        current_pos: Point = self.get_position()
        current_path_index = self.closest_point_index(current_pos)
        current_speed: float = self.get_speed()
        current_heading: float = self.get_heading_theta()
        heading_vector = (math.sin(current_heading), math.cos(current_heading))

        # Find lookahead information
        steering_lookahead: Point = self.point_at_distance(current_path_index, self.STEERING_LOOKEAHAD_DISTANCE)[0]
        v_lookeahead_time = self.MIN_TIME_LOOKAHEAD + current_speed * self.MAX_COMFORTABLE_DECELERATION
        velocity_lookahead, goal_v, time_to_v_lookahead = self.point_at_time(current_path_index, 1)

        # Extract goal acceleration, steering information
        goal_accel = (goal_v - current_speed) / max(time_to_v_lookahead,0.01)
        
        # limit accel to acceptable range
        goal_accel = min(goal_accel, self.MAX_COMFORTABLE_ACCELERATION)
        goal_accel = max(goal_accel, -self.MAX_SAFE_DECELERATION)

        # Calculate steering angle
        vector_to_lookahead = (steering_lookahead.y - current_pos.y, steering_lookahead.x - current_pos.x)
        # normalize vector
        dist_to_lookahead = max(math.sqrt(vector_to_lookahead[0]**2 + vector_to_lookahead[1]**2), 0.01)
        vector_to_lookahead = (vector_to_lookahead[0] / dist_to_lookahead, vector_to_lookahead[1] / dist_to_lookahead)
        # calculate angle between heading and lookahead using dot product
        steering_angle = math.acos((vector_to_lookahead[0] * heading_vector[0]) + (vector_to_lookahead[1] * heading_vector[1]))
        # calcualte sign of angle between heading and lookahead using cross product (2d)
        angle_sign = vector_to_lookahead[1] * heading_vector[0] - vector_to_lookahead[0] * heading_vector[1]
        # # flip sign if angle is negative
        if angle_sign < 0:
            steering_angle = -steering_angle

        # log heading vector, lookahead vector
        # self.get_logger().info("Heading vector: %s" % str(heading_vector))
        # self.get_logger().info("Lookahead vector: %s" % str(vector_to_lookahead))

        # self.get_logger().info("Throttle position: %f" % self.accel_to_throttle(goal_accel))
        # self.get_logger().info("Steering position: %f" % self.steering_angle_to_wheel(steering_angle))
        # self.get_logger().info("Brake position: %f" % self.accel_to_brake(goal_accel))


        self.throttle_pub.publish(PeddlePosition(
            data= self.accel_to_throttle(goal_accel)
        ))

        self.brake_pub.publish(PeddlePosition(
            data= self.accel_to_brake(goal_accel)
        ))

        self.steering_pub.publish(SteeringPosition(
            data= self.steering_angle_to_wheel(steering_angle)
        ))
    
    def closest_point_index(self, pos: Point) -> int:
        """
        Returns the index of the closest point in the path to the given point
        """
        path = self.cached_path.points

        min_dist: float = math.inf
        min_index = 0
        for i in range(len(path)):
            pt = path[i]
            d = (pt.x-pos.x)**2+(pt.y-pos.y)**2
            if d < min_dist:
                min_index = i
                min_dist = d
        return min_index

    def point_at_time(self, start_index: int, horizon: float):
        """
        Returns the (point, speed, time_to_point) at the given time (seconds) into the future, starting at the given index.
        Time follows idealized path velocities.
        """
        path = self.cached_path.points
        time_elapsed = 0
        prev_point: TrajectoryPoint = path[start_index]
        for i in range(1, len(path)):
            point_i: TrajectoryPoint = path[(start_index + i) % len(path)]
            dx = point_i.x - prev_point.x
            dy = point_i.y - prev_point.y
            d = math.sqrt(dx*dx + dy*dy)
            v = (prev_point.vx + point_i.vx) / 2
            
            if v == 0 or (time_elapsed + d/v >= horizon):
                return (
                    Point(x=prev_point.x,
                        y=prev_point.y,
                        z=0.0),
                    v, time_elapsed)
            
            time_elapsed += d / v

    def point_at_distance(self, start_index: int, distance: float):
        """
        Returns the (point, speed) at the given distance, starting at the given index.
        Time follows idealized path velocities.
        """
        path = self.cached_path.points
        dist_elapsed = 0
        prev_point: TrajectoryPoint = path[start_index]
        for i in range(1, len(path)):
            point_i: TrajectoryPoint = path[(start_index + i) % len(path)]
            dx = point_i.x - prev_point.x
            dy = point_i.y - prev_point.y
            d = math.sqrt(dx*dx + dy*dy)
           
            dist_elapsed += d

            if dist_elapsed >= distance:
                return (
                    Point(x=point_i.x,
                        y=point_i.y,
                        z=0.0),
                    point_i.vx)


    def get_heading_theta(self) -> Vector3:
        """
        Returns the heading vector of the vehicle
        """
        o = self.cached_odometry.pose.pose.orientation
        return self.quaternion_to_euler(o.x, o.y, o.z, o.w)[2]+(math.pi)

    def get_position(self) -> Point:
        """
        Returns the current position of the vehicle
        """
        pose = self.cached_odometry.pose.pose
        return Point(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z
        )

    def get_velocity(self) -> Vector3:
        """
        Returns the current velocity of the vehicle
        """
        return self.cached_odometry.twist.twist.linear

    def get_speed(self) -> float:
        """
        Returns the current speed of the vehicle
        """
        v = self.get_velocity()
        return math.sqrt(v.x*v.x + v.y*v.y+v.z*v.z)

    def steering_angle_to_wheel(self, angle: float) -> float:
        """
        Converts a steering angle to a steering position
        """
        return float(min(max(angle, -self.MAX_STEERING_ANGLE), self.MAX_STEERING_ANGLE))

    def accel_to_throttle(self, accel: float) -> float:
        """
        Converts an acceleration to a throttle position
        """
        return float(max(0, accel / 2.0))

    def accel_to_brake(self, accel: float) -> float:
        """
        Converts an acceleration to a brake position
        """
        return float(min(0, accel / 5.0))

    def __init__(self):
        super().__init__('unified_controller_node')

        # Subscribe to our path
        self.path_sub = self.create_subscription(
            Trajectory,
            '/planning/outgoing_trajectory',
            self.paths_cb,
            10
        )
        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/carla/odom',
            self.odom_cb,
            10
        )
        # Publishers for throttle, brake, and steering
        self.throttle_pub = self.create_publisher(
            PeddlePosition,
            '/command/throttle_position',
            10
        )
        self.brake_pub = self.create_publisher(
            PeddlePosition,
            '/command/brake_position',
            10
        )
        self.steering_pub = self.create_publisher(
            SteeringPosition,
            '/command/steering_position',
            10
        )
        self.command_viz_pub = self.create_publisher(
            Marker,
            '/command/viz',
            10
        )

        self.control_timer = self.create_timer(0.2, self.generate_commands)

    def quaternion_to_euler(self, x, y, z, w):
        import math
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        X = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        Y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        Z = math.atan2(t3, t4)

        return X, Y, Z
        
    def get_distance(pose: PoseWithCovarianceStamped, pt: Point):
        current_pos = pose.pose.pose.position
        return math.sqrt(math.pow((current_pos.x-pt.x),2)+math.pow((current_pos.y-pt.y),2))

def main(args=None):
    rclpy.init(args=args)

    unified_controller_node = UnifiedController()

    rclpy.spin(unified_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    unified_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()