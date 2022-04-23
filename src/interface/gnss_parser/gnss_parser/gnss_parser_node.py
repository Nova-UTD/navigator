#!/usr/bin/python

'''
Nova at UT Dallas, 2022

The Navigator Simulation Bridge for CARLA

The goal is to mimick Hail Bopp as much as possible.

Targetted sensors:
- GNSS (GPS)
✓ IMU ()
- Front and rear Lidar
✓ Front  RGB camera
✓ Front depth camera
- CARLA ground truths for
    - Detected objects
    ✓ Car's odometry (position, orientation, speed)
    ✓ CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

Todos:
- Specific todos are dispersed in this script. General ones are here.
- Ensure all sensors publish in ROS coordinate system, NOT Unreal Engine's.

'''

from scipy import rand
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, Polygon, PolygonStamped, Point32
from voltron_msgs.msg import PeddlePosition, SteeringPosition, Obstacle3DArray, Obstacle3D, BoundingBox3D, PolygonArray
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry  # For GPS, ground truth
from std_msgs.msg import Bool, Header, Float32, ColorRGBA
import math
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_msgs
from tf2_ros import TransformException, TransformStamped
import pymap3d
from cv_bridge import CvBridge
import numpy as np
import ros2_numpy as rnp
from rclpy.node import Node
import rclpy

# GPS stuff
import pymap3d as pm

lat0 = 32.989487
lon0 = -96.750437
alt0 = 196.0
skip_time = 14  # seconds

gps_log_path = "/home/main/voltron/assets/bags/april16/putty20220416170158.log"
# outfile = open("converted_gps.csv", 'w')


class GnssLogPublisher(Node):

    def __init__(self):
        super().__init__('gnss_log_publisher')

        # Create our publishers
        # self.road_cloud_sub = self.create_subscription(
        #     PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
        # )

        self.gnss_pub = self.create_publisher(
            Odometry, '/sensors/gnss/odom', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.road_boundary = None
        self.gps_timer = self.create_timer(0.5, self.publish_next_gnss)
        self.log_file = open(gps_log_path, 'r').readlines()
        # outfile.write(f"x,y,z,u,v,speed,pos_acc,yaw_acc,speed_acc\n")

        self.idx = skip_time*2  # 2 hz

    def publish_next_gnss(self):
        if self.idx == len(self.log_file):
            self.idx = skip_time*2  # Loop infinitely
        line = self.log_file[self.idx]
        parts = line.split()
        # print(parts)
        lat = int(parts[0])/1e7
        lon = int(parts[1])/1e7
        alt = alt0

        x, y, z = pm.geodetic2enu(lat, lon, alt, lat0, lon0, alt0)
        compass_yaw = float(parts[2])/1e5
        yaw_deg = (compass_yaw-90)*-1
        if yaw_deg < 0.0:
            yaw_deg += 360.0
        elif yaw_deg > 360.0:
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
        speed_acc = float(parts[6])/1e3  # TODO: verify these scales...

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
        # u = speed*math.cos(yaw)
        # v = speed*math.sin(yaw)

        # outfile.write(f"{x},{y},{z},{u},{v},{speed},{pos_acc},{yaw_acc},{speed_acc}\n")
        self.idx += 1


def main(args=None):
    rclpy.init(args=args)

    gnss_log_publisher = GnssLogPublisher()

    rclpy.spin(gnss_log_publisher)


if __name__ == '__main__':
    main()
