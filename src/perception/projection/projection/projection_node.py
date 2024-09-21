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

# Message Imports
from rosgraph_msgs.msg import Clock

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2
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

        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        # Subscribes to camera
        self.camera_sub = self.create_subscription(
            Image, '/cameras/camera0', self.camera_callback, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        # Publishes lidar data
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/abc', 10)

    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """

        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec

    def camera_callback(self, camera_msg: Image):
        self.image = camera_msg
        
        # For testing. Remove later.
        msg = String()
        msg.data = 'Hello World camera'
        self.publisher_.publish(msg)
        self.get_logger().info('Received image %dx%d' % (self.image.width, self.image.height))
        

    def lidar_callback(self, lidar_msg: PointCloud2):
        """! 
            Hello comment
        """

        # Get LIDAR points into np array
        self.get_logger().info('Lidar size: %d' % lidar_msg.row_step*lidar_msg.height)
        lid_arr = np.frombuffer(lidar_msg.data, dtype=np.float32)
        lid_arr = np.reshape(lid_arr, (-1, 4))
        lid_arr = lid_arr[:,0:3]
        
        
        # Project LIDAR points onto image coordinates
        lid_arr_1 = np.hstack((lid_arr, np.ones((len(lid_arr),1))))
        cam_arr = (self.matrix @ lid_arr_1.T).T
        

        # # Convert point cloud message to numpy array
        # lidar_array = rnp.numpify(msg)

        # # Strip away the dtypes
        # lidar_array_raw = np.vstack(
        #     (lidar_array['x'], lidar_array['y'], lidar_array['z'])).T

        # For testing. Remove later.
        msg = String()
        msg.data = 'Hello World lidar'
        self.publisher_.publish(msg)
        self.get_logger().info('Lidar: %s\n%s' % (np.array2string(cam_arr), np.array2string(lid_arr)))


        # # Values determined by model
        # inputs = self.model.preprocess(lidar_msg)
        # predictions = self.model.predict(inputs)

        # # Object3DArray ROS2 msg w/ bounding box data
        # objecst3d_array = self.model.postprocess(predictions, 
        #     self.conf_thresh, self.nms_thresh)
        
        # # Return if no detections made
        # if objecst3d_array is None:
        #     objecst3d_array = Object3DArray()
        
        # # Attaches the header to the message
        # objecst3d_array.header.stamp = self.stamp
        # objecst3d_array.header.frame_id = lidar_msg.header.frame_id
        
        # # Publishes the Object3DArray msg
        # self.objects3d_pub.publish(objecst3d_array)

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



