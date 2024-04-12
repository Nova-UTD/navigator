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
from object_detector_3d.mmdetection3d_model import MMDetection3DModel

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
        self.declare_parameter('model', 'mmdetection3d') # mmdetection3d | complex_yolo
        self.declare_parameter('conf_thresh', 0.7) # 0.0 - 1.0
        self.declare_parameter('nms_thresh', 0.2) # 0.0 - 1.0

        # Get ROS2 parameters
        self.device = torch.device(self.get_parameter('device') \
            .get_parameter_value().string_value)
        self.conf_thresh = self.get_parameter('conf_thresh') \
            .get_parameter_value().double_value
        self.nms_thresh = self.get_parameter('nms_thresh') \
            .get_parameter_value().double_value
        model_type = self.get_parameter('model') \
            .get_parameter_value().string_value
        
        # Declared for publishing msgs 
        self.stamp = Time() 
        
        # Instantiates the model type
        if model_type == 'complex_yolo':
            self.model = ComplexYOLOv4Model(self.device)
        else:
            self.model = MMDetection3DModel(str(self.device))
        
        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        # Subscribes to clock for headers
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        
        # Publishes array of 3D objects
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
        """! Uses the lidar msg and a 3D object deteciton model to
            form 3D bouding boxes around objects. Then, publishes
            the boxes to a 'detected' topic.
        @param lidar_msg[PointCloud2]   The raw lidar point cloud.
        """

        # Values determined by model
        inputs = self.model.preprocess(lidar_msg)
        predictions = self.model.predict(inputs)

        # Object3DArray ROS2 msg w/ bounding box data
        objecst3d_array = self.model.postprocess(predictions, 
            self.conf_thresh, self.nms_thresh)
        
        # Return if no detections made
        if objecst3d_array is None:
            objecst3d_array = Object3DArray()
        
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



