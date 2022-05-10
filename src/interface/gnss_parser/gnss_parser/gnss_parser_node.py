#!/usr/bin/python

import math

# GPS stuff
import pymap3d as pm
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # For GPS, ground truth
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import Imu
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped, Vector3, TransformStamped

lat0 = 32.989487
lon0 = -96.750437
alt0 = 196.0

frequency = 2  # hz, messages published per second


class GnssParserNode(Node):

    cached_string = ""

    def __init__(self):
        super().__init__('gnss_parser_node')

        # Create our publishers
        # self.road_cloud_sub = self.create_subscription(
        #     PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
        # )

        self.gnss_pub = self.create_publisher(
            Odometry, '/sensors/gnss/odom', 10)

        self.filtered_odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.publish_odom_tf, 10)

        self.pose_pub = self.create_publisher(PoseStamped, '/initial_pose', 10)

        self.string_data_sub = self.create_subscription(
            StringMsg, '/serial/gnss', self.string_data_callback, 10)

        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.road_boundary = None
        self.gps_timer = self.create_timer(
            1.0 / frequency, self.publish_next_gnss)
        # outfile.write(f"x,y,z,u,v,speed,pos_acc,yaw_acc,speed_acc\n")

        self.prev_yaw = None

    def publish_odom_tf(self, msg: Odometry):
        pos = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        transl = Vector3(
            x=pos.x,
            y=pos.y,
            z=pos.z
        )
        tf = TransformStamped()
        tf.child_frame_id = msg.child_frame_id
        tf.header = msg.header
        tf.transform.translation = transl
        tf.transform.rotation = quat

        self.br.sendTransform(tf)

    def string_data_callback(self, msg):
        opcode: str = msg.data.split()[0] # First part of space-separated packet
        
        if (opcode == "IMU"):
            self.handleIMU(msg)
        elif (opcode == "GPS"):
            self.handle_gnss(msg)

    def handle_imu(self, msg: StringMsg):
        parts = msg.data.split()
        acc_x, acc_y, acc_z = parts[1:4] # Linear acceleration
        vel_x, vel_y, vel_z = parts[4:]  # Angular velocity
        imuMsg = Imu()
        imuMsg.angular_velocity = Vector3(
            x = vel_x,
            y = vel_y,
            z = vel_z
        )
        imuMsg.linear_acceleration = Vector3(
            x = acc_x,
            y = acc_y,
            z = acc_z
        )
        imuMsg.header.stamp = self.get_clock().now().to_msg()
        imuMsg.header.frame_id = 'arduino_imu'
        
        self.imu_pub.publish(imuMsg)

    def handle_gnss(self, msg: StringMsg):
        line = msg.data
        if (line == ""):
            return
        parts = line.split()
        # print(parts)
        lat = int(parts[0])/1e7
        lon = int(parts[1])/1e7
        alt = alt0

        x, y, z = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
        compass_yaw = float(parts[2])/1e5
        yaw_deg = (compass_yaw-90)*-1
        while yaw_deg < 0.0:
            yaw_deg += 360.0
        while yaw_deg > 360.0:
            yaw_deg -= 360.0
        yaw = yaw_deg * math.pi/180.0
        speed = float(parts[3])/1000

        odomMsg = Odometry()
        odomMsg.header.stamp = self.get_clock().now().to_msg()
        odomMsg.header.frame_id = "map"
        # Should this be base_link? Doesn't make a huge difference.
        odomMsg.child_frame_id = "odom"
        odomMsg.pose.pose.position.x = x
        odomMsg.pose.pose.position.y = y
        odomMsg.pose.pose.position.z = z

        # This is an attempt to prevent the orientation from suddenly
        # flipping when the car is stopped
        if speed < 1.0 and self.prev_yaw is not None:
            odomMsg.pose.pose.orientation.w = math.cos(self.prev_yaw/2)
            odomMsg.pose.pose.orientation.z = math.sin(self.prev_yaw/2) 
            odomMsg.twist.twist.linear.x = speed*math.cos(self.prev_yaw)
            odomMsg.twist.twist.linear.y = speed*math.sin(self.prev_yaw)
        else:
            odomMsg.pose.pose.orientation.w = math.cos(yaw/2)
            odomMsg.pose.pose.orientation.z = math.sin(yaw/2)
            odomMsg.twist.twist.linear.x = speed*math.cos(yaw)
            odomMsg.twist.twist.linear.y = speed*math.sin(yaw)

        if speed > 1.0:
            self.prev_yaw = yaw



        pos_acc = float(parts[4])/1e7
        yaw_acc = float(parts[5])/1e5
        speed_acc = float(parts[6])/1e3

        msg.pose.covariance = [pos_acc, 0.0, 0.0, 0.0, 0.0, 0.0,
                               0.0, pos_acc, 0.0, 0.0, 0.0, 0.0,
                               0.0, 0.0, pos_acc, 0.0, 0.0, 0.0,
                               0.0, 0.0, 0.0, yaw_acc, 0.0, 0.0,
                               0.0, 0.0, 0.0, 0.0, yaw_acc, 0.0,
                               0.0, 0.0, 0.0, 0.0, 0.0, yaw_acc]

        msg.twist.covariance = [speed_acc, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, speed_acc, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.gnss_pub.publish(msg)

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)
        # u = speed*math.cos(yaw)
        # v = speed*math.sin(yaw)


def main(args=None):
    rclpy.init(args=args)

    gnss_log_publisher = GnssParserNode()

    rclpy.spin(gnss_log_publisher)


if __name__ == '__main__':
    main()
