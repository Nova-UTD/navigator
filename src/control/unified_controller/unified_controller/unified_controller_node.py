#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2022 - https://github.com/Nova-UTD/navigator/
# Will Heitman - Will.Heitman@utdallas.edu
# Egan Johnson - Egan.Johnson@utdallas.edu

from typing import List
from voltron_msgs.msg import Trajectory, TrajectoryPoint, PeddlePosition, SteeringPosition
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

class PIDController():
    kp = 0
    ki = 0
    kd = 0

    integral = 0
    last_error = 0
    last_time = 0

    max_integral = 1

    def __init__(self, kp, ki, kd, max_integral):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_integral = max_integral
    
   
    def reset(self, time):
        """
            Resets the controller to an initial state, to prevent
            the controller from making a jump at the start of a new
            sequence.
        """
        self.integral = 0
        self.last_error = 0
        self.last_time = time

    def update(self, time, error):
        """
            Updates the controller with the given error and returns
            the control signal.
        """
        dt = time - self.last_time
        if dt == 0:
            return 0
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        derivative = (error - self.last_error) / dt
        self.last_error = error
        self.last_time = time
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

    

class UnifiedController(Node):

    VEHICLE_MASS_KG = 769.0
    ENGINE_POWER_W = 6500.0
    WHEEL_FRICTION_COEFFICIENT_LOWER = 0.4
    WHEEL_FRICTION_COEFFICIENT_UPPER = 0.7
    BRAKE_FORCE = WHEEL_FRICTION_COEFFICIENT_UPPER * VEHICLE_MASS_KG * 9.81 # a guess

    WHEEL_BASE = 3.4
    MAX_STEERING_ANGLE = 0.58294 # radians

    MAX_SAFE_DECELERATION = 8 # m/s^2
    MAX_COMFORTABLE_DECELERATION = 1 # m/s^2
    MAX_COMFORTABLE_ACCELERATION = 1
    MAX_LATERAL_ACCELERATION = 1 # radial acceleration on turns

    MIN_TIME_LOOKAHEAD = 10
    VELOCITY_LOOKEAHED_DISTANCE = 2 + 3.4 # the car tracks velocity this far ahead of the origin of its coordinate 
    # system. I.e. if the origin is at the front of the car and lookeahead is 1 meter, the car will stop 1 meter
    # before zone boundaries.
    STEERING_LOOKEAHAD_DISTANCE = 7 # meters


    throttle_controller: PIDController
    brake_controller: PIDController

    # Pid constants
    KP_THROTTLE = 0.25
    KI_THROTTLE = 0.005
    KD_THROTTLE = 0.0
    MAX_WINDUP_THROTTLE = 0.5

    KP_BRAKE = 0.6
    KI_BRAKE = 0.1
    KD_BRAKE = 0.0
    MAX_WINDUP_BRAKE = 10


    STATE_ACCEL = 0 # 0 for neither, 1 for accel, -1 for decel


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
        odom_time = self.get_odom_time()

        self.get_logger().info("Current speed: %f" % current_speed)

        # Find lookahead information
        steering_lookahead: TrajectoryPoint = self.point_at_distance(current_path_index, self.STEERING_LOOKEAHAD_DISTANCE)
        v_lookeahead_time = self.MIN_TIME_LOOKAHEAD + current_speed * self.MAX_COMFORTABLE_DECELERATION
        velocity_lookahead, goal_v, time_to_v_lookahead = self.point_at_time(current_path_index, v_lookeahead_time, self.VELOCITY_LOOKEAHED_DISTANCE)

        # Get goal steering angle
        vector_to_lookahead = (steering_lookahead.y - current_pos.y, steering_lookahead.x - current_pos.x)
        goal_steering_angle = self.pure_pursuit(current_heading, vector_to_lookahead)
        
        # limit goal velocity based on goal steering angle

        # Print strait-line distance to lookahead
        distance = (steering_lookahead.x - current_pos.x)**2 + (steering_lookahead.y - current_pos.y)**2
        distance = math.sqrt(distance)
        self.get_logger().info("Distance to lookahead: %f" % distance)

        # limit steering angle based on current velocity and vehicle limits
        max_steering_angle = self.MAX_STEERING_ANGLE
        if current_speed > 0.05:
            goal_curvature = goal_steering_angle / self.WHEEL_BASE
            max_steering_angle = max(max_steering_angle, self.MAX_LATERAL_ACCELERATION * self.WHEEL_BASE / current_speed)
        goal_steering_angle = min(goal_steering_angle, max_steering_angle)
        goal_steering_angle = max(goal_steering_angle, -max_steering_angle)

        throttle, brake = self.calculate_throttle_brake(current_speed, goal_v, time_to_v_lookahead, odom_time)

        self.throttle_pub.publish(PeddlePosition(
            data= float(throttle)
        ))

        self.brake_pub.publish(PeddlePosition(
            data= float(brake)
        ))

        self.steering_pub.publish(SteeringPosition(
            # Coordinate system is flipped from what I was doing math with
            data= -float(goal_steering_angle)
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

    def point_at_time(self, start_index: int, horizon: float, distance_lookeahead: float = 0):
        """
        Returns the (point, speed, time_to_point) at the given time (seconds) into the future, starting at the given index.
        Time follows idealized path velocities.
        """
        path = self.cached_path.points
        time_elapsed = 0

        prev_point = path[start_index]

        if(distance_lookeahead > 0):
            prev_point = self.point_at_distance(start_index, distance_lookeahead)
            start_index = self.index_at_distance(start_index, distance_lookeahead)[0]

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

        Unlike index_at_distance, interpolates between points, so the returned point
        may not be a vertice but will have an arclength very close to given distance.
        """
        overshoot_index, overshoot_distance = self.index_at_distance(start_index, distance)
        overshoot_distance -= distance
        undershoot_index = (overshoot_index + len(self.cached_path.points) - 1) % len(self.cached_path.points)

        overshoot_point : TrajectoryPoint = self.cached_path.points[overshoot_index]
        undershoot_point : TrajectoryPoint= self.cached_path.points[undershoot_index]

        dx = overshoot_point.x - undershoot_point.x
        dy = overshoot_point.y - undershoot_point.y
        d = math.sqrt(dx*dx + dy*dy)

        interpolation_coefficient = 0
        if(d > 0.01):
            interpolation_coefficient = overshoot_distance / d

        interpolated_speed = overshoot_point.vx * interpolation_coefficient + \
            undershoot_point.vx * (1 - interpolation_coefficient)
        
        interpolated_point = TrajectoryPoint(x = undershoot_point.x + interpolation_coefficient * dx,
                                            y = undershoot_point.y + interpolation_coefficient * dy,
                                            vx = interpolated_speed, vy = 0.0)
       
        return interpolated_point
       

    def index_at_distance(self, start_index: int, distance: float):
        """ Returns the index of the point at the given distance from the start point.
            Returns (index, distance), where distance is the arc distance from the start point.
            Since the path is discrete, this may be greater than the given distance.
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
                return ((start_index + i) % len(path), dist_elapsed)
            
            prev_point = point_i


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

    def calculate_throttle_brake(self, current_speed: float, goal_speed: float, time_to_goal: float, odom_stamp: float):
        """
            Calculates the (throttle, brake) needed to reach the given speed, given the current speed,
            the time to reach the goal speed, and the current throttle and brake positions.
        """
        if current_speed < goal_speed:
            return self.calculate_throttle(current_speed, goal_speed, time_to_goal, odom_stamp), 0
        else:
            return 0, self.calculate_brake(current_speed, goal_speed, time_to_goal, odom_stamp)

    def calculate_throttle(self, current_speed: float, goal_speed: float, time_to_goal: float, odom_stamp: float):
        """
            Calculates the throttle needed to reach the given speed, given the current speed,
            the time to reach the goal speed, and the current throttle and brake positions.
        """
        # If we have not been using throttle, the controller is out of date
        if self.STATE_ACCEL < 1:
            self.throttle_controller.reset(odom_stamp)
            self.STATE_ACCEL = 1

        # Calculate lower bound of acceleration from friction, to avoid over-throttling
        a_friction = -self.WHEEL_FRICTION_COEFFICIENT_LOWER * 9.81

        a_target = (goal_speed - current_speed) / max(time_to_goal, 0.01)
        a_target = min(self.MAX_COMFORTABLE_ACCELERATION, a_target)

        a_active = a_target - a_friction

        # Assume throttle is linear with engine power.
        # p = m * v * a
        # throttle = p / pmax
        # However, while physically correct, this doesn't help us when velocity is 0.
        # Rely on the PID controller for this.
        throttle_theoretical = (self.VEHICLE_MASS_KG * current_speed * a_active) / self.ENGINE_POWER_W
        throttle = throttle_theoretical + self.throttle_controller.update(odom_stamp, a_target)
        return throttle

    def calculate_brake(self, current_speed: float, goal_speed: float, time_to_goal: float, odom_stamp: float):
        """
            Calculates the brake needed to reach the given speed, given the current speed,
            the time to reach the goal speed, and the current throttle and brake positions.
        """
        if self.STATE_ACCEL > -1:
            self.brake_controller.reset(odom_stamp)
            self.STATE_ACCEL = -1

        deceleration = (current_speed - goal_speed) / max(time_to_goal, 0.01)

        # Special case: stopping 
        if (abs(goal_speed) < 0.1 and abs(current_speed) < 0.1):
            # transition to braking to avoid jolt
            return 1.0 - abs(current_speed)
        
        return self.brake_controller.update(odom_stamp, deceleration)

    def pure_pursuit(self, heading_angle, offset_vector):
        """
        Pure pursuit calculator
        
        heading_direction: the direction of the heading vector
        offset_vector (x, y): vector from car origin to lookahead, in the same coordinate systems
            as heading_direction

        Reference
        https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf

        Returns steering angle
        """
        # rotate offset by heading angle to get offset in the car's coordinate system
        transformed_offset = [0, 0]
        transformed_offset[0] = offset_vector[0] * math.cos(heading_angle) - offset_vector[1] * math.sin(heading_angle)
        transformed_offset[1] = offset_vector[0] * math.sin(heading_angle) + offset_vector[1] * math.cos(heading_angle)

        mag_offset = math.sqrt(offset_vector[0] * offset_vector[0] + offset_vector[1] * offset_vector[1])

        curvature = 2* transformed_offset[0] / mag_offset**2

        # translate angle to 
        return curvature * self.WHEEL_BASE

    def __init__(self):
        super().__init__('unified_controller_node')

        self.throttle_controller = PIDController(self.KP_THROTTLE, self.KI_THROTTLE, self.KD_THROTTLE, self.MAX_WINDUP_THROTTLE)
        self.brake_controller = PIDController(self.KP_BRAKE, self.KI_BRAKE, self.KD_BRAKE, self.MAX_WINDUP_BRAKE)

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

    def get_odom_time(self):
        return self.cached_odometry.header.stamp.sec + self.cached_odometry.header.stamp.nanosec / 1e9
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