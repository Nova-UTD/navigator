"""
Package:   projection
Filename:  projection_node.py
Author:    David Homiller
Email:     david.homiller@utdallas.edu
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

# Message Imports
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

class ProjectionNode(Node):

    def __init__(self):
        super().__init__("projection_node")
        
        self.declare_parameter('seg_topic', '/semantics/semantic0')
        self.declare_parameter('camera_info_topic', '/carla/hero/rgb_center/camera_info')

        self.timer = self.create_timer(0.02, self.timer_cb) # should i use a timer? what should the period be?
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)

        # Subscribes to camera info
        camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_center/camera_info', self.camera_info_cb, 10)
        
        # Subscribes to semantic segmented image topic
        segmentation_sub = self.create_subscription(
            Image, '/semantics/semantic0', self.segmentation_cb, 10)
        
        # Subscribes to clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        self.cam_arr = None
        self.cam_model = None

    def timer_cb(self):
        pass

    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """

        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec

    def camera_info_cb(self, msg: CameraInfo):
        """Sets camera info for right camera

        Args:
            msg (CameraInfo)

        Returns:
            None
        """
        if self.cam_model is not None:
            return  # Already set, skip
        self.cam_model = image_geometry.PinholeCameraModel()
        self.cam_model.fromCameraInfo(msg)

    def lidar_callback(self, lidar_msg: PointCloud2):
        # Get LIDAR points into np array
        lid_arr = np.frombuffer(lidar_msg.data, dtype=np.float32)
        lid_arr = np.reshape(lid_arr, (-1, 4))
        lid_arr = lid_arr[:,0:3]
        

        t = TransformStamped()

        try:
            t = self.tf_buffer.lookup_transform(
                'hero/rgb_center', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frameawefew: {ex}')
            return

        # Rotate to camera frame
        q = t.transform.rotation
        tf_rotation: R = R.from_quat([q.x, q.y, q.z, q.w])
        cam_arr = tf_rotation.apply(lid_arr)

        # Translate to camera frame
        cam_arr += [t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z]

        # Only keep points in front of the camera
        cam_arr = cam_arr[cam_arr[:, 2] > 0]              
                             
        self.cam_arr = cam_arr

    

    def segmentation_cb(self, camera_msg: Image):
        if self.cam_arr is None:
            return
        if self.cam_model is None:
            return

        # 3x4 camera matrix
        P = np.asarray(self.cam_model.projectionMatrix()) 

        # LIDAR points (with 1s appended)
        cam_arr_1 = np.hstack((self.cam_arr, np.ones((len(self.cam_arr),1)))) 

        # convert LIDAR points to pixel coordinates
        pix_arr = (P @ cam_arr_1.T).T
        pix_arr[:,0:2] = (pix_arr[:,0:2].T / pix_arr[:,2]).T
        #pix_arr = pix_arr.astype(int)

        self.image = camera_msg

        # CV image for visualization. Remove later. Shows the segmentation image
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        except CvBridgeError as e:
            print(e)        

        # CV image for visualization. Remove later. Shows the LIDAR points
        for pt in pix_arr:
            color = max((int(255-5*pt[2]), 0))
            cv2.circle(cv_image, (int(pt[0]),int(pt[1])), 3, (color,0,color), -1)
        
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        

def main(args=None):
    rclpy.init(args=args)

    projection_node = ProjectionNode()

    rclpy.spin(projection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    projection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



