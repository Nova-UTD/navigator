# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Header
from autoware_auto_msgs.msg import VehicleKinematicState, VehicleControlCommand
from autoware_auto_msgs.msg import Trajectory
from autoware_auto_msgs.msg import TrajectoryPoint
from geometry_msgs.msg import PoseStamped, Point, PoseWithCovarianceStamped, PointStamped, Quaternion
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker
import math
import tf2_ros
import collections


class Translator(Node):

    def __init__(self):

        super().__init__('translator')
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/localization/ndt_pose',
            self.pose_cb,
            10)
        self.path_sub = self.create_subscription(
            Trajectory,
            '/planning/trajectory',
            self.trajectory_cb,
            10)

        self.command_pub = self.create_publisher(
            VehicleControlCommand,
            '/vehicle/vehicle_command',
            10)
        self.command_viz_pub_ = self.create_publisher(
            Marker, '/vehicle/vehicle_command_viz', 10)

        self.cached_pose = PoseWithCovarianceStamped()
        # self.path_pub_ = self.create_publisher(Path, '/planning/path_viz', 10)
        # self.command_viz_pub_ = self.create_publisher(Marker, '/vehicle/vehicle_command_viz', 10)
        # self.hack_header = Header()

        # self.subscription  # prevent unused variable warning

    def pose_cb(self, msg):
        self.cached_pose = msg
    
    def trajectory_cb(self, msg: Trajectory):
        print(len(msg.points))
        if (len(msg.points)<4):
            print("You've arrived! Skipping steering.")
            return;
        
        # Construct vehicle command message
        command = VehicleControlCommand()
        command.stamp = msg.header.stamp;



        if len(msg.points) > 12:
            steering_theta = self.get_theta(msg.points[10], msg.points[11], self.cached_pose,
                                            A=0.2, B=1.0, C=0.4)
            command.front_wheel_angle_rad = steering_theta;
            self.command_pub.publish(command)


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

    def get_theta(self, routept_A: TrajectoryPoint, routept_B: TrajectoryPoint, pose: PoseWithCovarianceStamped, A, B, C):
        Vector = collections.namedtuple('Vector', ['magnitude', 'angle'])
        
        current_heading_quat = pose.pose.pose.orientation;
        
        
        current_heading_rads = self.quaternion_to_euler(current_heading_quat.x, current_heading_quat.y, current_heading_quat.z, current_heading_quat.w)[2]
        # self.get_logger().info("\nHeading: {}\n".format(current_heading_rads))
        v_0 = Vector(magnitude=1, angle=current_heading_rads) #unit vector
        position = pose.pose.pose.position
        VEHICLE_LENGTH = 2 #meter
        position.x += VEHICLE_LENGTH * math.cos(current_heading_rads)
        position.y += VEHICLE_LENGTH * math.sin(current_heading_rads)
        
        v_1_theta = math.atan2(routept_A.y-position.y, routept_A.x-position.x)
        v_1 = Vector(magnitude=1, angle=v_1_theta)

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
        self.viz_vectors(v_0_x, v_0_y, v_1_x, v_1_y, v_2_x, v_2_y, pose.header, position.x, position.y, v_res_x, v_res_y)
        return(result_theta*-1)
        

    def get_distance(pose: PoseWithCovarianceStamped, PathPoint: TrajectoryPoint):
        current_pos = pose.pose.pose.position
        return math.sqrt(math.pow((current_pos.x-PathPoint.x),2)+math.pow((current_pos.y-PathPoint.y),2))

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
        self.command_viz_pub_.publish(v_0_marker)

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
        self.command_viz_pub_.publish(v_1_marker)

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
        self.command_viz_pub_.publish(v_2_marker)

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
        self.command_viz_pub_.publish(v_r_marker)


def main(args=None):
    rclpy.init(args=args)

    translator = Translator()

    rclpy.spin(translator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
