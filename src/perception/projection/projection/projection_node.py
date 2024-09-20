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
import torch

# Ros Imports
import rclpy
from rclpy.node import Node

# For testing. Remove later.
from std_msgs.msg import String

# Message Imports
from rosgraph_msgs.msg import Clock

from builtin_interfaces.msg import Time
from sensor_msgs.msg import Image, PointCloud2
from navigator_msgs.msg import Object3DArray

class LidarDetectionNode(Node):

    def __init__(self):
        super().__init__("projection_node")

        # Declare default ROS2 node parameters
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('model', 'mmdetection3d')
        self.declare_parameter('conf_thresh', 0.7) 
        self.declare_parameter('nms_thresh', 0.2)

        # Get ROS2 parameters
        self.device = torch.device(self.get_parameter('device') \
            .get_parameter_value().string_value)
        self.conf_thresh = self.get_parameter('conf_thresh') \
            .get_parameter_value().double_value
        self.nms_thresh = self.get_parameter('nms_thresh') \
            .get_parameter_value().double_value
        model_type = self.get_parameter('model') \
            .get_parameter_value().string_value
        config_path = self.get_parameter('config_path') \
            .get_parameter_value().string_value
        checkpoint_path = self.get_parameter('checkpoint_path') \
            .get_parameter_value().string_value
        
        # For testing. Remove later.
        self.publisher_ = self.create_publisher(String, 'terror', 10)

        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        # Subscribes to camera
        self.camera_sub = self.create_subscription(
            Image, '/camera/camera0', self.camera_callback, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)

        # Publishes lidar data
        self.lidar_pub = self.create_publisher(
            PointCloud2, '/', 10)

    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """


        # For testing. Remove later.
        msg = String()
        msg.data = 'Hello World'
        self.publisher_.publish(msg)

        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec

    def camera_callback(self, lidar_msg: Image):
        pass

    def lidar_callback(self, lidar_msg: PointCloud2):
        """! Uses the lidar msg and a 3D object deteciton model to
            form 3D bouding boxes around objects. Then, publishes
            the boxes to a 'detected' topic.
        @param lidar_msg[PointCloud2]   The raw lidar point cloud.
        """

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

    projection_node = LidarDetectionNode()

    rclpy.spin(projection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    projection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



