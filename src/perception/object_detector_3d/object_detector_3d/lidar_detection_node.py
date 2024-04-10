"""
Package:   object_detector_3d
Filename:  lidar_detection_node.py
Author:    Gueren Sanford
Email:     guerensanford@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License
Description of what this file does, what inputs it takes and what it outputs or accomplishes
"""

# Python Imports
import torch, math
import numpy as np
from time import time # DEBUG

# Local Import
from object_detector_3d.complex_yolov4_model import ComplexYOLOv4Model

# Ros Imports
import rclpy
import ros2_numpy as rnp
from rclpy.node import Node
import sensor_msgs_py.point_cloud2 as pc2

# Message definitions
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from navigator_msgs.msg import Object3DArray

class LidarDetectionNode(Node):

    def __init__(self):
        """! Initializes the node."""
        super().__init__("lidar_detection_node")

        # Declare default ROS2 node parameters
        self.declare_parameter('device', 'cuda:1') # cpu | cuda:{gpu num}
        self.declare_parameter('conf_thresh', 0.4) # 0.0 - 1.0
        self.declare_parameter('nms_thresh', 0.2) # 0.0 - 1.0
        self.declare_parameter('down_sample', False)

        # Get ROS2 parameters
        self.device = torch.device(self.get_parameter('device') \
            .get_parameter_value().string_value)
        self.conf_thresh = self.get_parameter('conf_thresh') \
            .get_parameter_value().double_value
        self.nms_thresh = self.get_parameter('nms_thresh') \
            .get_parameter_value().double_value
        self.down_sample = self.get_parameter('down_sample') \
            .get_parameter_value().bool_value
        
        # Instantiates the model
        self.model = ComplexYOLOv4Model(self.device)
        self.stamp = Time() # For published msgs
        
        # Subcribes to filtered lidar
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        
        # Instantiates publishers
        self.objects3d_pub = self.create_publisher(
            Object3DArray, 'detected/objects3d', 10)

    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """
        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec

    def lidar_callback(self, lidar_msg: PointCloud2):
        """!
        For now, just prints the lidar point cloud and turns it into a numpy array.
        @param lidar_msg[PointCloud2]   The filtered lidar point cloud.
        """

        # 608x608 birds eye view rgb map of points
        rgb_map = self.model.preprocess(lidar_msg)
        # Turned into a tensor for the model
        input_bev = torch.tensor(rgb_map, device=self.device).float() \
            .unsqueeze(0)

        # The model outputs [1, boxes, x y w l classes]
        outputs = self.model.predict(input_bev)

        objecst3d_array = self.model.postprocess(rgb_map, outputs, 
            self.conf_thresh, self.nms_thresh)
        
        # Return if no detections made
        if objecst3d_array is None:
            return
        
        # Attaches the header to the message
        objecst3d_array.header.stamp = self.stamp
        objecst3d_array.header.frame_id = lidar_msg.header.frame_id
        
        # Publishes the Object3DArray msg
        self.objects3d_pub.publish(objecst3d_array)

def main(args=None):
    rclpy.init(args=args)

    lidar_detection_node = LidarDetectionNode()

    rclpy.spin(lidar_detection_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lidar_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



