"""
Package:   frame_tf_service
Filename:  frame_tf_service.py
Authors:   David Homiller, Saishravan Muthukrishnan, (AI was used for help with some debugging, math, learning information about ROS2, and packages.xml and CmakeLists.txt additions)
Email:     david.homiller@utdallas.edu, saishravan.muthukrishnan@utdallas.edu
Copyright: 2021, Nova UTD
License:   MIT License

TODO: Description
"""

# CV2 import for testing. Remove later.
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Imports
import numpy as np

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import image_geometry
from rclpy.serialization import serialize_message, deserialize_message


# Message Imports
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Vector3

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from frame_tf_serv.srv import FrameTF


class FrameTFService(Node):
    
    def __init__(self):
        super().__init__('FrameTFService')
        
        self.declare_parameter('seg_topic', '/semantics/semantic0')
        self.srv = self.create_service(
            FrameTF,
            'frame_tf',
            self.callback
        )
        self.serialized_message = None
        self.timer = self.create_timer(0.02, self.timer_cb) # should i use a timer? what should the period be?
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ex = None

        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        
        # holds LIDAR transform stamp
        self.t = None

        # Subscribes to camera info
        rgb_center_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_center/camera_info', self.rgb_center_camera_info_cb, 10)
        rgb_left_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_left/camera_info',self.rgb_left_camera_info_cb, 10)
        rgb_right_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_right/camera_info',self.rgb_right_camera_info_cb,10)
        rgb_back_camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_back/camera_info', self.rgb_back_camera_info_cb,10)
        # Subscribes to semantic segmented image topic

        
        # Subscribes to clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        
        self.cam_arr = None
        
        # stores camera info
        self.rgb_center_cam_model = None
        self.rgb_left_cam_model = None
        self.rgb_right_cam_model = None
        self.rgb_back_cam_model = None

    def timer_cb(self):
            pass
        
    def rgb_center_camera_info_cb(self, msg: CameraInfo):
        """Sets camera info for center camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        self.rgb_center_cam_model = msg
        
    
    def rgb_right_camera_info_cb(self, msg: CameraInfo):
        """Sets camera info for right camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        self.rgb_right_cam_model = image_geometry.PinholeCameraModel().fromCameraInfo(msg) 

    def rgb_left_camera_info_cb(self, msg: CameraInfo):
        """Sets camera info for left camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        self.rgb_left_cam_model = image_geometry.PinholeCameraModel().fromCameraInfo(msg)    
    
    def rgb_back_camera_info_cb(self, msg: CameraInfo):
        """Sets camera info for back camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        self.rgb_back_cam_model = image_geometry.PinholeCameraModel().fromCameraInfo(msg)    
    
    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """

        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec
        
    def lidar_callback(self, lidar_msg: PointCloud2):
        """ stores LIDAR transform information
        
        Args:
            lidar_msg (PointCloud2)
            
        Returns:
            None
        """
        
        # Get LIDAR points into np array
        self.lid_arr = np.frombuffer(lidar_msg.data, dtype=np.float32)
        self.lid_arr = np.reshape(lid_arr, (-1, 4))
        self.lid_arr = lid_arr[:,0:3]

    def attach_depth(self, camera):
        """ Applies camera intrinsics/extrinsics to LIDAR point cloud and stores it
        
        Args:
            camera (string)
        Returns:
            None
        """
        t = TransformStamped()
        try:
            t = self.tf_buffer.lookup_transform(
                'hero/'+camera, 'base_link', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=2.0))
            #print("found transform!")
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frameawefew: {ex}')
            return

        # Rotate to camera frame
        q = t.transform.rotation
        tf_rotation: R = R.from_quat([q.x, q.y, q.z, q.w])
        cam_arr = tf_rotation.apply(self.lid_arr)

        # Translate to camera frame
        cam_arr += [t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z]

        # Only keep points in front of the camera
        cam_arr = cam_arr[cam_arr[:, 2] > 0]              
        self.cam_arr = cam_arr
    
    def camImage_to_world(self,camera,stamp):
        """ Converts all pixels in a camera image into world coordinates and stores in a blob
        
        Args:
            camera (string)
            stamp (Time)
        Returns:
            None
        """
        while True:
            try:
                # get rotation matrix
                r_wc = self.tf_buffer.lookup_transform('hero/'+camera, 'base_link', stamp,timeout=rclpy.duration.Duration(seconds=2.0)).transform.rotation
                R_wc = R.from_quat([r_wc.x, r_wc.y, r_wc.z, r_wc.w]).as_matrix()
                
                # get translation matrix
                t_wc = self.tf_buffer.lookup_transform('hero/'+camera, 'base_link', stamp,timeout=rclpy.duration.Duration(seconds=2.0)).transform.translation
                t_wc = np.array([t_wc.x,t_wc.y,t_wc.z])
                
                # get tranformed LIDAR coords                
                attach_depth(camera)
                
                #  Apply camera rotation and translation to transformed LIDAR points
                x_w = R_wc.T@self.cam_arr.T + t_wc.reshape(3,1)
                
                # store as blob
                self.serialized_message = x_w.tobytes()
                break;
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform to camera frameawefew: {ex}')
                continue

    def world_to_pixel(self, camera, stamp, world_coord):
        """ Converts given world coordinates into pixel coordinates and stores in a blob
        
        Args:
            camera (string)
            stamp (Time)
            world_coord (3 Tuple)
        Returns:
            None
        """
        while True:
            try:
                # get rotation matrix
                r_lc = self.tf_buffer.lookup_transform('hero/'+camera, 'map', stamp, timeout=rclpy.duration.Duration(seconds=2.0)).transform.rotation
                R_lc = R.from_quat([r_lc.x, r_lc.y, r_lc.z, r_lc.w]).as_matrix()
                
                # get translation matrix 
                t_wc = self.tf_buffer.lookup_transform('hero/'+camera, 'map', stamp, timeout=rclpy.duration.Duration(seconds=2.0)).transform.translation
                t_wc = np.array([t_wc.x,t_wc.y,t_wc.z])
                
                # choose correct set of camera intrinsics
                selected_camera = None
                if camera == "rgb_center":
                    selected_camera = self.rgb_center_cam_model
                elif camera == "rgb_left":
                    selected_camera = self.rgb_left_cam_model
                elif camera == "rgb_right":
                    selected_camera = self.rgb_right_cam_model
                elif camera == "rgb_back":
                    selected_camera = self.rgb_back_cam_model
                
                fx = selected_camera.k[0]
                cx = selected_camera.k[2]
                fy = selected_camera.k[4]
                cy = selected_camera.k[5]
                
                # transform coordinates
                p_cam = R_lc*world_coord + t_wc
                
                # divide transformed world coordinates into pixel coordinates and apply intrinsics
                x = world_coord[0] / world_coord[2]
                y = world_coord[1] / world_coord[2];
                x_pixel = fx * x + cx
                y_pixel = fy * y + cy
                coords = (x_pixel, y_pixel)
                #print(coords)
                
                # store as blob
                self.serialized_message = bytes(coords);
                break;
            
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not convert world to pixel: {ex}')
                continue
                
    
    def callback(self,request, response):
        """ Calls selected transformation function based on recieved message parameters, and returns serialized output
        
        Args:
            camera (string)
            stamp (Time)
            world_coord (3 Tuple)
        Returns:
            None
        """
        # execute frame_tf operation based on selection
        if len(request.camera_name) > 1:
            if request.cam_to_world == 1:
                self.camImage_to_world(request.camera_name,request.stamp)
                response.coords = self.serialized_message
                response.tf_success = True
            elif request.world_to_pixel == 1:
                self.world_to_pixel (request.camera_name, request.stamp, (request.x, request.y, request.z))
                response.coords = self.serialized_message
                response.tf_success = True
            else:
                response.tf_success = False
                print("transform failed")
        else:
            print("Camera name not provided: ", request.cam_to_world)
        return response            

def main(args=None):
    rclpy.init(args=args)
    node = FrameTFService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
