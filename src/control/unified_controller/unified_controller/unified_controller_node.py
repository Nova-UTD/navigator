#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2022 - https://github.com/Nova-UTD/navigator/
# Will Heitman - Will.Heitman@utdallas.edu
# Egan Johnson - Egan.Johnson@utdallas.edu

from typing import List
from voltron_msgs.msg import Trajectory, TrajectoryPoint, PeddlePosition, SteeringPosition
from std_msgs.msg import Float32
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point

# For noise testing only
# from numpy import random

def quaternion_to_euler(x, y, z, w):
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

def yield_arrays_consecutively(arr1, arr2, arr1_start = 0, arr2_start = 0):
    for i in range(arr1_start, len(arr1)):
        yield arr1[i]
    for i in range(arr2_start, len(arr2)):
        yield arr2[i]

class PIDController():
    kp = 0
    ki = 0
    kd = 0

    integral = 0
    last_error = 0
    last_time = 0
    derivative = 0

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
        self.derivative = 0
        self.last_error = 0
        self.last_time = time

    def update(self, time, error):
        """
            Updates the controller with the given error and returns
            the control signal.
        """
        dt = time - self.last_time
        if dt > 0:
            self.derivative = (error - self.last_error) / dt
        self.integral += error * dt
        if self.integral > self.max_integral:
            self.integral = self.max_integral
        elif self.integral < -self.max_integral:
            self.integral = -self.max_integral
        
        self.last_error = error
        self.last_time = time
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * self.derivative)

    

class UnifiedController(Node):

    WHEEL_BASE = 3.4
    MAX_STEERING_ANGLE = 0.506 # radians

    TOP_SPEED = 9.0 # m/s
    MAX_THROTTLE = 0.5
    THROTTLE_RAMP = 0.3 # 1 / throttle_ramp = seconds to reach max throttle from 0

    VELOCITY_LOOKEAHED_DISTANCE = 4 + 3.4 # the car tracks velocity this far ahead of the origin of its coordinate 
    # system. I.e. if the origin is at the front of the car and lookeahead is 1 meter, the car will stop 1 meter
    # before zone boundaries.
    STEERING_LOOKEAHAD_DISTANCE = 7 # meters

    # Pid constants
    KP_TB = 1.0
    KI_TB = 0.0
    KD_TB = 0.0 # 0.3
    MAX_WINDUP_TB = 5

    KP_THROTTLE = 0.25
    KP_BRAKE = 0.45

    MAX_ACCEPTABLE_MESSAGE_DELAY = 0.5 # seconds

    tb_controller : PIDController

    cached_odometry: Odometry = None
    cached_path: Trajectory = None

    # Update these variables once per loop for and use for 
    # convienience in all methods
    velocity : Vector3 
    speed : float 
    position : TrajectoryPoint # trajectory point for convenience working with combining the position and path points
    heading_theta : float # 0 is x axis (?). In radians
    path : List[TrajectoryPoint] # saved version of cached_path.points in case cached_path is updated mid-calculations
    stamp_time : float = 0.0
    current_path_index : int

    # Moving average variables
    last_steering_position : float = 0.0
    last_throttle_position : float = 0.0
    last_brake_position : float = 1.0
    
    current_time : float = 0.0
    time_between_updates: float = 0.0
    last_published_throttle : float = 0.0

    STEERING_LAST_WEIGHT = 0.0 # weight of the last steering position in the moving average. i.e for 0.25
    # the output is 0.25 * last_steering_position + 0.75 * current_steering_position 
    THROTTLE_LAST_WEIGHT = 0.0

    def paths_cb(self, msg: Trajectory):
        self.cached_path = msg

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg

    def update_state(self):
        """ Pulls information from cached messages and updates
        state convenience variables """
        odo : Odometry = self.cached_odometry
        path : Trajectory = self.cached_path

        self.path = path.points
        new_stamp_time = odo.header.stamp.sec + odo.header.stamp.nanosec/1e9
        if(abs(new_stamp_time - self.stamp_time) > 1):
            self.tb_controller.reset(new_stamp_time)
        self.stamp_time = new_stamp_time
        
        t = self.get_clock().now().seconds_nanoseconds()
        t = t[0] + t[1]/1e9
        self.time_between_updates = t - self.current_time
        self.current_time = t
        if(self.time_between_updates > 1):
            self.time_between_updates = 0

        self.velocity = odo.twist.twist.linear
        self.speed = (self.velocity.x**2 + self.velocity.y**2)**0.5
        
        pos = odo.pose.pose.position
        self.position = TrajectoryPoint(
            x = pos.x,
            y = pos.y, 
            vx = self.speed,
            vy = 0.0)

        o = odo.pose.pose.orientation
        self.heading_theta = quaternion_to_euler(o.x, o.y, o.z, o.w)[2]+(math.pi)

        self.current_path_index = self.closest_point_index(self.position)

    def publish_commands(self, throttle, brake, steering):
          # publish commands
        thr = min((throttle, self.MAX_THROTTLE, self.last_published_throttle + self.THROTTLE_RAMP * self.time_between_updates))
        thr = max((thr, 0.0))
        self.throttle_pub.publish(Float32(
            data= float(thr)
        ))
        self.last_published_throttle = thr

        self.brake_pub.publish(Float32(
            data= float(brake)
        ))

        self.steering_pub.publish(Float32(
            # Coordinate system is flipped from what I was doing math with
            data= float(steering)
        ))


    def generate_commands(self):

        if (self.cached_odometry is None) or (self.cached_path is None) or (len(self.cached_path.points) == 0):
            self.publish_commands(0, 1, 0)
            return

        self.update_state()

        if abs(self.current_time - self.stamp_time) > self.MAX_ACCEPTABLE_MESSAGE_DELAY:
            # self.publish_commands(0, 1, 0)
            self.get_logger().error("Time between messages exceeds acceptable limit: PLEASE EMERGENCY STOP")
            # return
            pass

        # get lookahead distances based on reachable path
        reachable_dist = self.reachable_distance(self.current_path_index, max(self.STEERING_LOOKEAHAD_DISTANCE, self.VELOCITY_LOOKEAHED_DISTANCE))
        v_look_dist = min(self.VELOCITY_LOOKEAHED_DISTANCE, reachable_dist)
        s_look_dist = max(min(self.STEERING_LOOKEAHAD_DISTANCE, reachable_dist), self.STEERING_LOOKEAHAD_DISTANCE / 2)

        # Get goal steering angle
        steering_lookahead: TrajectoryPoint = self.point_at_distance(self.current_path_index, s_look_dist)
        vector_to_lookahead = (steering_lookahead.y - self.position.y, steering_lookahead.x - self.position.x)
        goal_steering_angle = self.pure_pursuit(self.heading_theta, vector_to_lookahead)  

        goal_steering_angle = min(goal_steering_angle, self.MAX_STEERING_ANGLE)
        goal_steering_angle = max(goal_steering_angle, -self.MAX_STEERING_ANGLE)

        # Visualize
        lookahead_arrow = Marker()
        lookahead_arrow.header.stamp = self.get_clock().now().to_msg()
        lookahead_arrow.header.frame_id = 'map'
        lookahead_arrow.action = Marker.ADD
        lookahead_arrow.color = ColorRGBA(
            r=1.0,
            g=0.0,
            b=1.0,
            a=1.0,
        )
        lookahead_arrow.id = 1
        lookahead_arrow.ns = 'pure_pursuit'
        lookahead_arrow.scale.x = 1.0
        lookahead_arrow.points.append(Point(
            x=self.position.x,
            y=self.position.y
        ))
        lookahead_arrow.points.append(Point(
            x=steering_lookahead.x,
            y=steering_lookahead.y
        ))
        lookahead_arrow.frame_locked = True
        lookahead_arrow.type = Marker.ARROW
        self.lookahead_arrow_viz_pub.publish(lookahead_arrow)

        # Get throttle and brake
        v_look = self.point_at_distance(self.current_path_index, v_look_dist)
        target_velocity = v_look.vx
        throttle, brake = self.calculate_throttle_brake(target_velocity, self.stamp_time)

        # apply moving average to steering and throttle
        steering_angle = self.last_steering_position * self.STEERING_LAST_WEIGHT + goal_steering_angle * (1 - self.STEERING_LAST_WEIGHT)
        steering_angle = min(max(steering_angle, -self.MAX_STEERING_ANGLE), self.MAX_STEERING_ANGLE)
        self.last_steering_position = steering_angle

        if(steering_angle < 0):
            self.get_logger().info("Left")
        else:
            self.get_logger().info("Right")

        throttle = self.last_throttle_position * self.THROTTLE_LAST_WEIGHT + throttle * (1 - self.THROTTLE_LAST_WEIGHT)
        brake = self.last_brake_position * self.THROTTLE_LAST_WEIGHT + brake * (1 - self.THROTTLE_LAST_WEIGHT)
        

        self.last_throttle_position = throttle
        self.last_brake_position = brake
        if(self.last_brake_position < 0.05):
            self.last_brake_position = 0.0
        if(self.last_throttle_position < 0.05):
            self.last_throttle_position = 0.0
        
        if(brake > 0.0):
            throttle = 0.0

        self.publish_commands(throttle, brake, steering_angle)

        self.get_logger().info("Current velocity: %f Target velocity: %f Throttle: %f Brake: %f" %(self.speed, target_velocity, throttle,  brake))
    
    def closest_point_index(self, pos) -> int:
        """
        Returns the index of the closest point in the path to the given position
        """
        min_dist: float = math.inf
        min_index = 0
        for i in range(len(self.path)):
            pt = self.path[i]
            d = (pt.x-pos.x)**2+(pt.y-pos.y)**2
            if d < min_dist:
                min_index = i
                min_dist = d
        return min_index
      
    def point_at_distance(self, start_index: int, distance: float):
        """
        Returns the (point, speed) at the given distance, starting at the given index.

        Unlike index_at_distance, interpolates between points, so the returned point
        may not be a vertice but will have an arclength very close to given distance.

        If we run out of path, returns the last point in the path.
        """
        overshoot_index, overshoot_distance = self.index_at_distance(start_index, distance)
        overshoot_distance -= distance

        # If overshoot is negative, we are at the end of the path
        if overshoot_distance < 0:
            # Return the last point with no interpolation
            return self.path[-1]

        undershoot_index = max(0, start_index - 1)

        overshoot_point : TrajectoryPoint = self.path[overshoot_index]
        undershoot_point : TrajectoryPoint= self.path[undershoot_index]

        dx = overshoot_point.x - undershoot_point.x
        dy = overshoot_point.y - undershoot_point.y
        d = math.sqrt(dx*dx + dy*dy)

        if(d < 0.01):
            return undershoot_point

        interpolation_coefficient = 1 - (overshoot_distance / d)

        # "interpolated"
        interpolated_speed = overshoot_point.vx
        
        interpolated_point = TrajectoryPoint(x = undershoot_point.x + interpolation_coefficient * dx,
                                            y = undershoot_point.y + interpolation_coefficient * dy,
                                            vx = interpolated_speed, vy = 0.0)
       
        return interpolated_point
       

    def reachable_distance(self, start_index: int, max_distance : float) -> float:
        """
            Finds what distance ahead of start_index is reachable in any amount of 
            time. Anything following a period of 0 velocity or past the end of the
            path is not reachable, so it will return the distance up to that point.

            Stops searching at max_distance and returns that distance. 
        """
        distance = 0
        prev_point = self.path[start_index]
        for i in range(start_index +1, len(self.path)):
            pi = self.path[i]
            if(pi.vx == 0) and (prev_point.vx == 0):
                break

            distance += math.sqrt((pi.x - prev_point.x)**2 + (pi.y - prev_point.y)**2)
            if distance > max_distance:
                break
        
        return min(distance, max_distance)

    def index_at_distance(self, start_index: int, distance: float):
        """ Returns the index of the point at the given distance from the start point.
            Returns (index, distance), where distance is the arc distance from the start point.
            Since the path is discrete, this may be greater than the given distance.
            If there is not enough path, return the last index and distance will be smaller than given.
         """ 
        path = self.path
        dist_elapsed = 0
        prev_point: TrajectoryPoint = path[start_index]
        for i in range(1, len(path) - start_index - 1):
            point_i: TrajectoryPoint = path[start_index + i]
            dx = point_i.x - prev_point.x
            dy = point_i.y - prev_point.y
            d = math.sqrt(dx*dx + dy*dy)
            dist_elapsed += d

            if dist_elapsed >= distance:
                return (start_index + i, dist_elapsed)
            
            prev_point = point_i
        
        return (len(path) - 1, dist_elapsed)


    def steering_angle_to_wheel(self, angle: float) -> float:
        """
        Converts a steering angle to a steering position
        """
        return float(min(max(angle, -self.MAX_STEERING_ANGLE), self.MAX_STEERING_ANGLE))

    def calculate_throttle_brake(self, target_v : float, odom_stamp: float):
        """
            Calculates the (throttle, brake) needed to reach the given speed, given the current speed,
            the time to reach the goal speed, and the current throttle and brake positions.
        """

        self.get_logger().info("Calculating throttle/brake for target speed %f" % target_v)

        # special case: full brake when stopped
        if abs(target_v) < 0.2 and abs(self.speed) < 0.2:
            self.tb_controller.reset(odom_stamp)
            return (0.0, 1.0 - self.speed)

        #psudeo_force = target_v / self.TOP_SPEED
        psudeo_force = self.tb_controller.update(odom_stamp, target_v - self.speed)

        if psudeo_force > 0:
            return (min(1.0, psudeo_force * self.KP_THROTTLE), 0.0) 
        if psudeo_force <= 0:
            return (0.0, min(1.0, -psudeo_force * self.KP_BRAKE))


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

        curvature = 0
        if(mag_offset > 0.01):
            curvature = 2* transformed_offset[0] / mag_offset**2

        # translate angle to 
        return curvature * self.WHEEL_BASE

    def __init__(self):
        super().__init__('unified_controller_node')

        self.tb_controller = PIDController(self.KP_TB, self.KI_TB, self.KD_TB, self.MAX_WINDUP_TB)

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
            '/gnss_odom',
            self.odom_cb,
            10
        )
        # Publishers for throttle, brake, and steering
        self.throttle_pub = self.create_publisher(
            Float32,
            '/command/throttle_position',
            10
        )
        self.brake_pub = self.create_publisher(
            Float32,
            '/command/brake_position',
            10
        )
        self.steering_pub = self.create_publisher(
            Float32,
            '/command/steering_position',
            10
        )
        self.desired_speed_pub = self.create_publisher(
            Float32,
            '/control/desired_speed',
            10
        )
        self.lookahead_arrow_viz_pub = self.create_publisher(
            Marker, '/viz/lookahead', 10
        )
        self.steering_angle_viz_pub = self.create_publisher(
            Marker, '/viz/steering_angle', 10
        )
        self.control_timer = self.create_timer(0.2, self.generate_commands)


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