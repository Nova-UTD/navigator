#!/usr/bin/python

# Unified Controller for Throttle, Brake, and Steering
# Nova 2022 - https://github.com/Nova-UTD/navigator/
# Will Heitman - Will.Heitman@utdallas.edu

from typing import List
from voltron_msgs.msg import Trajectory, PeddlePosition, SteeringPosition
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import tf2_ros
import collections

class UnifiedController(Node):

    cached_odometry = Odometry()
    cached_path = Trajectory()
    a_offset = 40
    b_offset = 4
    pointA_idx = 40
    pointB_idx = pointA_idx+1

    def paths_cb(self, msg: Trajectory):
        self.cached_path = msg

    def generate_commands(self):
        path: Trajectory = self.cached_path
        

        throttle, brake, steering = (0.0, 0.0, 0.0)

        if (len(path.points) == 0):
            self.get_logger().warn("No path recieved.")
            steering = 0.0
            return
        else:# len(path.points) > self.pointB_idx:
            self.pointA_idx = (self.a_offset + self.closest_point(path.points, self.cached_odometry.pose.pose.position)) % len(path.points)
            self.pointB_idx = (self.pointA_idx + self.b_offset) % len(path.points)
            steering_theta = self.get_theta(path.points[self.pointA_idx], path.points[self.pointB_idx], self.cached_odometry,
                                            A=0.2, B=0.7, C=0.4)
            steering = data=steering_theta * 0.5
        
            

        current_twist = self.cached_odometry.twist.twist.linear
        current_speed = math.sqrt(
            current_twist.x ** 2 + current_twist.y ** 2
        )
        #self.get_logger().info("Received path with {} points".format(len(path.points)))
        desired_speed = min(5.0 - 3*steering, path.points[self.pointA_idx].vx) # m/s
        #self.get_logger().info(f"point A index {self.pointA_idx}, desired = {desired_speed}, current = {current_speed}")
        

        if (current_speed < desired_speed):
            throttle = 0.5
            brake = 0.0
        elif (current_speed - desired_speed > 0.5):
            throttle = 0.0
            brake = 0.3

        self.throttle_pub.publish(PeddlePosition(
            data=throttle
        ))

        self.brake_pub.publish(PeddlePosition(
            data=brake
        ))

        self.steering_pub.publish(SteeringPosition(
            data=steering
        ))

        #self.get_logger().info("S: {}".format(steering))

    def odom_cb(self, msg: Odometry):
        self.cached_odometry = msg

    def closest_point(self, path: List[Point], pos: Point) -> int:
        min_dist: float = math.inf
        min_index = 0
        for i in range(len(path)):
            pt = path[i]
            d = (pt.x-pos.x)**2+(pt.y-pos.y)**2
            if d < min_dist:
                min_index = i
                min_dist = d
        return min_index

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

    def get_theta(self, routept_A: Point, routept_B: Point, odom: Odometry, A, B, C):
        Vector = collections.namedtuple('Vector', ['magnitude', 'angle'])
        
        current_heading_quat = odom.pose.pose.orientation;
        
        current_heading_rads = self.quaternion_to_euler(current_heading_quat.x, current_heading_quat.y, current_heading_quat.z, current_heading_quat.w)[2]+(math.pi)
        # self.get_logger().info("\nHeading: {}\n".format(current_heading_rads))

        # v0 simply represents our car's heading
        v_0 = Vector(magnitude=1, angle=current_heading_rads) #unit vector
        position = odom.pose.pose.position
        VEHICLE_LENGTH = 2 # meters
        position.x += VEHICLE_LENGTH * math.cos(current_heading_rads)
        position.y += VEHICLE_LENGTH * math.sin(current_heading_rads)
        
        # v1 is vector from current pos to first route point
        v_1_theta = math.atan2(routept_A.y-position.y, routept_A.x-position.x)
        v_1 = Vector(magnitude=1, angle=v_1_theta)

        # v2 is vector from first to second route point
        v_2_theta = math.atan2(routept_B.y-routept_A.y,routept_B.x-routept_A.x)
        v_2 = Vector(magnitude=1, angle=v_2_theta)

        # multiply by our constants
        v_0_mag = v_0.magnitude * A
        v_1_mag = v_1.magnitude * B
        v_2_mag = v_2.magnitude * C

        # Calculate coordinates for each vector (I know there's a better way...)
        v_1_x = v_1_mag*math.cos(v_1.angle)
        v_2_x = v_2_mag*math.cos(v_2.angle)
        v_0_x = v_0_mag*math.cos(v_0.angle)
        v_1_y = v_1_mag*math.sin(v_1.angle)
        v_2_y = v_2_mag*math.sin(v_2.angle)
        v_0_y = v_0_mag*math.sin(v_0.angle)
        # Now add
        v_res_x = v_1_x+v_2_x+v_0_x
        v_res_y = v_1_y+v_2_y+v_0_y
        v_res_theta = math.atan2(v_res_y, v_res_x)
        v_res_magnitude = math.sqrt(math.pow(v_res_x,2)+math.pow(v_res_y,2))

        # Normalize
        v_r_norm = Vector(magnitude=1, angle= v_res_theta)
        v_r_norm_x = v_res_x/v_res_magnitude
        v_r_norm_y = v_res_y/v_res_magnitude
        v_0_norm_x = math.cos(v_0.angle)
        v_0_norm_y = math.sin(v_0.angle)
        # Dot product
        dp = (v_r_norm_x*v_0_norm_x)+(v_r_norm_y*v_0_norm_y)
        # self.get_logger().info("\nW: <{},{}>\n".format(v_r_norm_x,v_r_norm_y))
        # RESULT
        result_theta = v_res_theta-v_0.angle
        if (result_theta>math.pi):
            result_theta = result_theta-(2*math.pi)
        if (result_theta<-math.pi):
            result_theta = result_theta+(2*math.pi)
        # self.get_logger().info("\n\Turn: {}\n".format(result_theta))
        self.viz_vectors(v_0_x, v_0_y, v_1_x, v_1_y, v_2_x, v_2_y, odom.header, position.x, position.y, v_res_x, v_res_y)
        return(result_theta*-1)
        
    def get_distance(pose: PoseWithCovarianceStamped, pt: Point):
        current_pos = pose.pose.pose.position
        return math.sqrt(math.pow((current_pos.x-pt.x),2)+math.pow((current_pos.y-pt.y),2))

    def viz_vectors(self, v_0_x, v_0_y, v_1_x, v_1_y, v_2_x, v_2_y, header, o_x, o_y, v_r_x, v_r_y):
        v_0_marker = Marker()
        v_0_marker.header = header
        v_0_marker.header.frame_id = "map"
        v_0_marker.type = Marker.ARROW
        v_0_marker.ns = 'steering_command_vectors'
        v_0_marker.id = 3
        v_0_marker.action = Marker.ADD
        pa = Point()
        pa.x = o_x
        pa.y = o_y
        pb = Point()
        pb.x = v_0_x*10+o_x
        pb.y = v_0_y*10+o_y
        points = [pa, pb]
        v_0_marker.points = points
        v_0_marker.scale.x = 1.0
        v_0_marker.scale.y = 2.0
        v_0_marker.color.a = 1.0
        v_0_marker.color.r = 1.0
        v_0_marker.color.g = 0.0
        v_0_marker.color.b = 1.0
        self.command_viz_pub.publish(v_0_marker)

        v_1_marker = Marker()
        v_1_marker.header = header
        v_1_marker.header.frame_id = "map"
        v_1_marker.type = Marker.ARROW
        v_1_marker.ns = 'steering_command_vectors'
        v_1_marker.id = 4
        v_1_marker.action = Marker.ADD
        pc = Point()
        pc.x = pb.x
        pc.y = pb.y
        pd = Point()
        pd.x = v_1_x*10+pb.x
        pd.y = v_1_y*10+pb.y
        points = [pc, pd]
        v_1_marker.points = points
        v_1_marker.scale.x = 1.0
        v_1_marker.scale.y = 2.0
        v_1_marker.color.a = 1.0
        v_1_marker.color.r = 0.0
        v_1_marker.color.g = 0.0
        v_1_marker.color.b = 1.0
        self.command_viz_pub.publish(v_1_marker)

        v_2_marker = Marker()
        v_2_marker.header = header
        v_2_marker.header.frame_id = "map"
        v_2_marker.type = Marker.ARROW
        v_2_marker.ns = 'steering_command_vectors'
        v_2_marker.id = 5
        v_2_marker.action = Marker.ADD
        pe = Point()
        pe.x = pd.x
        pe.y = pd.y
        pf = Point()
        pf.x = v_2_x*10+pd.x
        pf.y = v_2_y*10+pd.y
        points = [pe, pf]
        v_2_marker.points = points
        v_2_marker.scale.x = 1.0
        v_2_marker.scale.y = 2.0
        v_2_marker.color.a = 1.0
        v_2_marker.color.r = 1.0
        v_2_marker.color.g = 0.0
        v_2_marker.color.b = 0.0
        self.command_viz_pub.publish(v_2_marker)

        v_r_marker = Marker()
        v_r_marker.header = header
        v_r_marker.header.frame_id = "map"
        v_r_marker.type = Marker.ARROW
        v_r_marker.ns = 'steering_command_vectors'
        v_r_marker.id = 6
        v_r_marker.action = Marker.ADD
        pg = Point()
        pg.x = o_x
        pg.y = o_y
        ph = Point()
        ph.x = v_r_x*10+o_x
        ph.y = v_r_y*10+o_y
        points = [pg, ph]
        v_r_marker.points = points
        v_r_marker.scale.x = 1.0
        v_r_marker.scale.y = 2.0
        v_r_marker.color.a = 1.0
        v_r_marker.color.r = 0.0
        v_r_marker.color.g = 1.0
        v_r_marker.color.b = 0.0
        self.command_viz_pub.publish(v_r_marker)



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