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

import cv2
import time
from scipy import rand
from sensor_msgs.msg import PointCloud2, Image
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, PoseWithCovarianceStamped
from voltron_msgs.msg import Obstacle3DArray, Obstacle3D, BoundingBox3D, BoundingBox2D, Obstacle2D, Obstacle2DArray
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
from scipy.spatial.transform import Rotation as R

# ZED stuff
import pyzed.sl as sl
svo_path = "/home/main/voltron/assets/bags/april16/HD720_SN34750148_17-02-06_trimmed_3s5.svo"
camera_id = 0
zed = sl.Camera()
USE_BATCHING = True

# Image format conversion

'''
CHECKLIST
=========

Publish to ROS:
- [x] RGB image from left camera (15 Hz)
- [x] Depth map (15 Hz)
- [-] Point cloud -- Skipped, too slow for now
- [x] Array of detected objects
- [x] Pose data (with sensor fusion of optical odom) (max Hz)

'''


class ZedUnpacker(Node):

    def __init__(self):
        super().__init__('zed_unpacker')
        self.declare_parameter('use_real_camera', 'true')

        # Create our publishers
        # self.road_cloud_sub = self.create_subscription(
        #     PointCloud2, '/lidar/semantic/road', self.calculate_bias, 10
        # )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/sensors/zed/pose', 10)

        self.left_rgb_pub = self.create_publisher(
            Image, 'sensors/zed/left_rgb', 10
        )

        self.depth_img_pub = self.create_publisher(
            Image, 'sensors/zed/depth_img', 10
        )
        self.pcd_pub = self.create_publisher(
            PointCloud2, 'sensors/zed/depth_cloud', 10
        )
        self.object_pub_3d = self.create_publisher(
            Obstacle3DArray, 'sensors/zed/obstacle_array_3d', 10)

        self.object_pub_2d = self.create_publisher(
            Obstacle2DArray, 'sensors/zed/obstacle_array_2d', 10)

        self.br = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        init_parameters = sl.InitParameters()
        if self.get_parameter('use_real_camera').get_parameter_value().string_value == 'true':
            init_parameters.set_from_camera_id(0)
        else:
            init_parameters.set_from_svo_file(svo_path)
        

        # Use the ROS coordinate system for all measurements
        init_parameters.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP_X_FWD
        init_parameters.coordinate_units = sl.UNIT.METER  # Set units in meters
        init_parameters.depth_minimum_distance = 1.0
        init_parameters.depth_minimum_distance = 40.0
        init_parameters.svo_real_time_mode = True
        zed = sl.Camera()
        status = zed.open(init_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            print(repr(status))
            self.get_logger().error(f"{repr(status)}")
            exit()

        # Enable pos tracking
        tracking_params = sl.PositionalTrackingParameters()
        zed.enable_positional_tracking(tracking_params)

        # Enable object detection
        batch_parameters = sl.BatchParameters()
        batch_parameters.enable = False
        obj_param = sl.ObjectDetectionParameters(
            batch_trajectories_parameters=batch_parameters)

        obj_param.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX
        # Defines if the object detection will track objects across images flow.
        obj_param.enable_tracking = True
        zed.enable_object_detection(obj_param)

        # Configure object detection runtime parameters
        obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
        detection_confidence = 60
        obj_runtime_param.detection_confidence_threshold = detection_confidence
        # To select a set of specific object classes
        obj_runtime_param.object_class_filter = [
            sl.OBJECT_CLASS.VEHICLE, sl.OBJECT_CLASS.PERSON, sl.OBJECT_CLASS.ANIMAL]
        # To set a specific threshold
        obj_runtime_param.object_class_detection_confidence_threshold = {
            sl.OBJECT_CLASS.PERSON: detection_confidence}

        runtime = sl.RuntimeParameters()
        camera_pose = sl.Pose()

        camera_info = zed.get_camera_information()

        py_translation = sl.Translation()
        pose_data = sl.Transform()

        image = sl.Mat()
        depth = sl.Mat()
        point_cloud = sl.Mat()
        objects = sl.Objects()  # Structure containing all the detected objects
        while True:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
                # Read left RGB image
                zed.retrieve_image(image, sl.VIEW.LEFT)
                # Retrieve depth matrix. Depth is aligned on the left RGB image
                zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
                # Retrieve colored point cloud
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)

                # Retrieve the detected objects
                zed.retrieve_objects(objects, obj_runtime_param)

                # Get the timestamp at the time the image was captured
                timestamp = zed.get_timestamp(sl.TIME_REFERENCE.CURRENT)

                # Get current pose from tracker

                tracking_state = zed.get_position(
                    camera_pose, sl.REFERENCE_FRAME.WORLD)

                if tracking_state == sl.POSITIONAL_TRACKING_STATE.OK:
                    self.publish_pose(camera_pose)
                else:
                    self.get_logger().warning("Positional tracking not available.")

                self.publish_zed_img(image)
                self.publish_depth_img(depth)
                self.publish_object_boxes(objects)

                # Disable for now... It's too slow
                # self.publish_depth_cloud(point_cloud)

                # print(translation.get())

                # print("Image resolution: {0} x {1} || Image timestamp: {2}\n".format(svo_image.get_width(), svo_image.get_height(),
                #  timestamp.get_milliseconds()))

                # Get frame count
                svo_position = zed.get_svo_position()
            elif zed.grab() == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                print("SVO end has been reached. Looping back to first frame")
                zed.set_svo_position(0)

            else:
                print(zed.grab())

    def publish_zed_img(self, mat: sl.Mat):
        np_mat = mat.get_data()
        # msg = rnp.msgify(Image, np_mat, 'bgra8')
        msg = self.br.cv2_to_imgmsg(np_mat)
        msg.header.stamp = self.get_clock().now().to_msg()
        # Not strictly accurate, should be in left frame...
        msg.header.frame_id = 'zed2_camera_center'
        self.left_rgb_pub.publish(msg)

    def publish_depth_img(self, mat: sl.Mat):
        tic = time.time()
        np_mat = mat.get_data()
        # msg = rnp.msgify(Image, np_mat, 'bgra8')
        msg = self.br.cv2_to_imgmsg(np_mat)
        msg.header.stamp = self.get_clock().now().to_msg()
        # Not strictly accurate, should be in left frame...
        msg.header.frame_id = 'zed2_camera_center'
        self.depth_img_pub.publish(msg)
        # print(time.time()-tic)

    def publish_depth_cloud(self, mat: sl.Mat):
        tic = time.time()
        np_mat = mat.get_data()
        # print(np_mat)
        np_mat = np_mat[np.logical_not(np.isnan(np_mat[:, :, 0]))]
        np_mat.dtype = [
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('rgba', np.float32)
        ]
        print(np_mat)
        msg = rnp.msgify(PointCloud2, np_mat)
        msg.header.stamp = self.get_clock().now().to_msg()
        # Not strictly accurate, should be in left frame...
        msg.header.frame_id = 'zed2_camera_center'
        self.pcd_pub.publish(msg)
        print(time.time()-tic)

    def publish_object_boxes(self, objects: sl.Objects):
        obj_array = Obstacle3DArray()
        obj_2d_array = Obstacle2DArray()
        print(f"Objects: {len(objects.object_list)}")
        for object in objects.object_list:
            obj_msg = Obstacle3D()
            obj_2d_msg = Obstacle2D()
            if object.label == sl.OBJECT_CLASS.PERSON:
                obj_msg.label = Obstacle3D.PEDESTRIAN
                obj_2d_msg.label = Obstacle3D.PEDESTRIAN
            elif object.label == sl.OBJECT_CLASS.VEHICLE:
                if object.sublabel == sl.OBJECT_SUBCLASS.BICYCLE:
                    obj_msg.label = Obstacle3D.BIKE
                    obj_2d_msg.label = Obstacle3D.BIKE
                else:
                    obj_msg.label = Obstacle3D.CAR
                    obj_2d_msg.label = Obstacle3D.CAR
            else:
                obj_msg.label = Obstacle3D.OTHER
                obj_2d_msg.label = Obstacle3D.OTHER
            obj_msg.id = object.id
            obj_2d_msg.id = object.id
            obj_msg.confidence = object.confidence
            obj_2d_msg.confidence = object.confidence
            # print(object.bounding_box)
            obj_msg.velocity = Vector3(
                x=object.velocity[0],
                y=object.velocity[1],
                z=object.velocity[2]
            )
            # Add 3D box only if we have eight corners
            bbox_msg = BoundingBox3D()
            corners = []
            if len(object.bounding_box[:] == 8):
                for point in object.bounding_box[:]:

                    pt = Point(
                        x=point[0].item(),
                        y=point[1].item(),
                        z=point[2].item()
                    )
                    corners.append(pt)
                    # print(pt)
            print(corners)
            if len(corners) == 8:
                bbox_msg.corners = corners
                print(bbox_msg.corners)
            obj_msg.bounding_box = bbox_msg
            obj_array.obstacles.append(obj_msg)

            # Finally, add 2D box
            bbox_msg_2d = BoundingBox2D()
            corners = []
            if len(object.bounding_box_2d[:] == 4):
                corners = object.bounding_box_2d[:]
                bbox_msg_2d.a[0] = corners[0][0].item()
                bbox_msg_2d.a[1] = corners[0][1].item()
                bbox_msg_2d.b[0] = corners[1][0].item()
                bbox_msg_2d.b[1] = corners[1][1].item()
                bbox_msg_2d.c[0] = corners[2][0].item()
                bbox_msg_2d.c[1] = corners[2][1].item()
                bbox_msg_2d.d[0] = corners[3][0].item()
                bbox_msg_2d.d[1] = corners[3][1].item()
            obj_2d_msg.bounding_box = bbox_msg_2d
            obj_2d_array.obstacles.append(obj_2d_msg)

        obj_array.header.stamp = self.get_clock().now().to_msg()
        obj_array.header.frame_id = 'base_link'
        obj_2d_array.header.stamp = self.get_clock().now().to_msg()
        obj_2d_array.header.frame_id = 'base_link'
        # obj_msg.velocity = obje
        # print("{} {}".format(object.id, object.position))
        self.object_pub_3d.publish(obj_array)
        self.object_pub_2d.publish(obj_2d_array)

    def publish_pose(self, pose: sl.Pose):
        rotation = pose.get_rotation_vector()
        translation = pose.get_translation().get()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # Hm...  We will set our state estimate UKF to differentiate this.
        msg.header.frame_id = 'map'

        # Set position
        msg.pose.pose.position.x = translation[0]
        msg.pose.pose.position.y = translation[1]
        # This should be zero... but it isn't
        msg.pose.pose.position.z = translation[2]

        # Set orientation
        quat = R.from_rotvec(rotation).as_quat()  # [x,y,z,w]
        msg.pose.pose.orientation.x = quat[0]
        msg.pose.pose.orientation.y = quat[1]
        msg.pose.pose.orientation.z = quat[2]
        msg.pose.pose.orientation.w = quat[3]

        self.pose_pub.publish(msg)


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
