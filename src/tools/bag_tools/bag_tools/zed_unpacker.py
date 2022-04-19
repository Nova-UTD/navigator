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

# ZED stuff
import pyzed.sl as sl
svo_path = "/home/main/voltron/assets/bags/april16/HD720_SN34750148_17-02-06_trimmed.svo"
zed = sl.Camera()


class ZedUnpacker(Node):

    def __init__(self):
        super().__init__('zed_unpacker')

        # Create our publishers
        # self.road_cloud_sub = self.create_subscription(
        #     PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
        # )

        self.gnss_pub = self.create_publisher(
            Odometry, '/sensors/gnss/odom', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        init_parameters = sl.InitParameters()
        init_parameters.set_from_svo_file(svo_path)

        # Use the ROS coordinate system for all measurements
        init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_parameters.coordinate_units = sl.UNIT.METER  # Set units in meters
        init_parameters.svo_real_time_mode = True

        zed = sl.Camera()
        status = zed.open(init_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            exit()

        tracking_params = sl.PositionalTrackingParameters()
        zed.enable_positional_tracking(tracking_params)

        runtime = sl.RuntimeParameters()
        camera_pose = sl.Pose()

        camera_info = zed.get_camera_information()

        py_translation = sl.Translation()
        pose_data = sl.Transform()

        svo_image = sl.Mat()
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                # Read right RGB image
                zed.retrieve_image(svo_image, sl.VIEW.RIGHT)
                # Get the timestamp at the time the image was captured
                timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)

                # Get current pose from tracker

                tracking_state = zed.get_position(
                    camera_pose, sl.REFERENCE_FRAME.WORLD)
                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    rotation = camera_pose.get_rotation_vector()
                    translation = camera_pose.get_translation(py_translation)
                    print(translation.get())

                # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(svo_image.get_width(), svo_image.get_height(),
                    #  timestamp.get_milliseconds()))

                # Get frame count
                svo_position = zed.get_svo_position()
            elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                print("SVO end has been reached. Looping back to first frame")
                zed.set_svo_position(0)

            else:
                print(zed.grab())

    # def publish_zed_img(self, mat: sl.Mat):


def main(args=None):
    rclpy.init(args=args)

    zed_unpacker = ZedUnpacker()

    rclpy.spin(zed_unpacker)
    zed.close()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    zed_unpacker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
