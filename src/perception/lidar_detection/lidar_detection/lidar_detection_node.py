"""
Package:   lidar_detection
Filename:  lidar_detection_node.py
Author:    Gueren Sanford
Email:     guerensanford@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License
Subscribes to raw lidar pointcloud data and publishes an array 3D object predictions. The 
model for the predictions can be chosen using the model parameter. The predictions can be
filtered using the node's prediction confidence threshold and the non-maximum supression
threshold (limits the intersection of boxes). 
"""

# Python Imports
import torch

# Local Import
from lidar_detection.complex_yolov4_model import ComplexYOLOv4Model
from lidar_detection.mmdetection3d_model import MMDetection3DModel

# Ros Imports
import rclpy
from rclpy.node import Node

# Message Imports
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from sensor_msgs.msg import PointCloud2
from navigator_msgs.msg import Object3DArray

class LidarDetectionNode(Node):

    def __init__(self):
        """! Initializes the node.
        @param device[str]   The device the model will run on. Choices:
            'cpu' (DEFAULT) | 'cuda:{NUMBER}'
        @param model[str]   The type of model making the detections. Choices:
            'mmdetection3d' (DEFAULT) | 'complex_yolo'
        @param conf_thresh[float]   The mininum confidence value accepted
            for bounding boxes. Choices: 0.7 (DEFAULT) | 0.0 - 1.0 
        @param nms_thresh[float]   The maximum accepted intersection accepted
            for bounding boxes. Choices 0.2 (DEFAULT) | 0.0 - 1.0 
        """
        super().__init__("lidar_detection_node")

        # Declare default ROS2 node parameters
        self.declare_parameter('device', 'cuda:0')
        self.declare_parameter('model', 'mmdetection3d')
        self.declare_parameter('conf_thresh', 0.7) 
        self.declare_parameter('nms_thresh', 0.2)
        self.declare_parameter('config_path',   # Paths to pretrained model and config
            './data/perception/configs/pointpillars_hv_secfpn_8xb6-160e_kitti-3d-3class.py')
        self.declare_parameter('checkpoint_path', 
            './data/perception/models/hv_pointpillars_secfpn_6x8_160e_kitti-3d-3class_20220301_150306-37dc2420.pth')

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
        
        # Declared for publishing msgs 
        self.stamp = Time() 
        
        # Instantiates the model type
        if model_type == 'complex_yolo':
            self.model = ComplexYOLOv4Model(self.device)
        else:
            self.model = MMDetection3DModel(config_path, checkpoint_path, str(self.device))
        
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



