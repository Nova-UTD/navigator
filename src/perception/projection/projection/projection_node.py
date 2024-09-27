"""
Package:   projection
Filename:  projection_node.py
Author:    David Homiller
Email:     david.homiller@utdallas.edu
Copyright: 2021, Nova UTD
License:   MIT License

TODO: Description
"""

# Python Imports
import numpy as np
import math

# Ros Imports
import rclpy
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as point_cloud2

# For testing. Remove later.
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import image_geometry

# Message Imports
from rosgraph_msgs.msg import Clock

from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from navigator_msgs.msg import Object3DArray

class ProjectionNode(Node):

    def __init__(self):
        super().__init__("projection_node")
        
        # Camera parameters. Currently set for CARLA
        fov = 70 # degrees
        img_width, img_height = 800, 600 # pixels
        c = np.array([0.7,-0.15,1.88]) # location of camera in CARLA coords
        r = np.eye(3) # rotation of camera

        # intrinsic matrix
        f = img_width / (2 * math.tan(fov*math.pi/360)) # focal length
        cu, cv = img_width/2, img_height/2 # center of image in pixel coordinates
        m1 = np.array([[f, 0, cu, 0],
                        [0, f, cv, 0],
                        [0, 0, 1, 0]]) 
        
        # extrinsic matrix
        t = - r@c
        m2 = np.hstack((r, t.reshape(-1, 1)))
        m2 = np.vstack((m2, [0,0,0,1]))

        # final matrix
        self.matrix = m1@m2

        # For testing. Remove later.
        self.publisher_ = self.create_publisher(String, '/terror', 10)
        self.timer = self.create_timer(5, self.timer_cb)
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)



        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        # Subscribes to camera
        self.camera_sub = self.create_subscription(
            Image, '/cameras/camera0', self.camera_callback, 10)

        # Subscribes to camera info
        camera_info_sub = self.create_subscription(
            CameraInfo, '/carla/hero/rgb_center/camera_info', self.camera_info_cb, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        # Publishes lidar data
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/abc', 10)

        self.cam_arr = None
        self.cam_model = None

    # For testing. Remove later.
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

    def camera_callback(self, camera_msg: Image):
        if self.cam_arr is None:
            return
        if self.cam_model is None:
            return
        # For testing. Remove later.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(camera_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        P = np.asarray(self.cam_model.projectionMatrix())
        cam_arr_1 = np.hstack((self.cam_arr, np.ones((len(self.cam_arr),1))))
        pix_arr = (P @ cam_arr_1.T).T
        pix_arr[:,0:2] = (pix_arr[:,0:2].T / pix_arr[:,2]).T
        #pix_arr = pix_arr.astype(int)
        # self.get_logger().info('hi %s' % np.array2string(pix_arr))
        for pt in pix_arr:
            #uv = self.cam_model.project3dToPixel(pt)
            if pt[0] > 1023 or pt[0] < 0:
                continue
            elif pt[1] > 511 or pt[1] < 0:
                continue
            color = max((int(255-5*pt[2]), 0))
            cv2.circle(cv_image, (int(pt[0]),int(pt[1])), 3, (color,0,color), -1)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

        # Project LIDAR points onto image coordinates
        
        # cam_arr[:,0:2] = (cam_arr[:,0:2].T / cam_arr[:,2]).T

        # only points in frame
        # cam_arr = cam_arr[cam_arr[:, 0] > 0] 
        # cam_arr = cam_arr[cam_arr[:, 1] > 0] 
        # cam_arr = cam_arr[cam_arr[:, 0] < 600] 
        # cam_arr = cam_arr[cam_arr[:, 1] < 600] 

        self.image = camera_msg
        

    def lidar_callback(self, lidar_msg: PointCloud2):
        """! 
            Hello comment
        """

        # Get LIDAR points into np array
        # self.get_logger().info('Lidar size: %d' % lidar_msg.row_step*lidar_msg.height)
        lid_arr = np.frombuffer(lidar_msg.data, dtype=np.float32)
        lid_arr = np.reshape(lid_arr, (-1, 4))
        lid_arr = lid_arr[:,0:3]
        

        t = TransformStamped()

        try:
            t = self.tf_buffer.lookup_transform(
                'hero/rgb_center', 'base_link', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frame: {ex}')
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



