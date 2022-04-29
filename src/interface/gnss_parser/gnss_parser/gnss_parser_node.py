#!/usr/bin/python

import math

# GPS stuff
import pymap3d as pm
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry  # For GPS, ground truth
from std_msgs.msg import String as StringMsg
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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.br = TransformBroadcaster(self)
        self.road_boundary = None
        self.gps_timer = self.create_timer(
            1.0 / frequency, self.publish_next_gnss)
        # outfile.write(f"x,y,z,u,v,speed,pos_acc,yaw_acc,speed_acc\n")

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
        self.cached_string = msg.data

    def publish_next_gnss(self):
        line = self.cached_string
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

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        # Should this be base_link? Doesn't make a huge difference.
        msg.child_frame_id = "odom"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = math.cos(yaw/2)
        msg.pose.pose.orientation.z = math.sin(yaw/2)

        msg.twist.twist.linear.x = speed*math.cos(yaw)
        msg.twist.twist.linear.y = speed*math.sin(yaw)

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
