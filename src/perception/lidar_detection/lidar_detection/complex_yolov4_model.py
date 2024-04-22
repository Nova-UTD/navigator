"""
Package:   lidar_detection
Filename:  complex_yolov4_model.py
Author:    Gueren Sanford
Email:     guerensanford@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License
Use the ComplexYOLOv4Model class to make object detections with LiDAR data in the format 
[x,y,z,intensity]. We use the model Complex Yolo to make the 
predictions, which requires a birds eye view map of the LiDAR data. 
With the predictions, feed it into the postprocess to filter out and 
format the results.
"""

# Python Imports
import random
import numpy as np
import ros2_numpy as rnp
import torch

# Model Imports
from lidar_detection.complex_yolov4.model.darknet2pytorch import Darknet
from lidar_detection.complex_yolov4.utils import kitti_bev_utils
from lidar_detection.complex_yolov4.utils.evaluation_utils import post_processing_v2
import  lidar_detection.complex_yolov4.utils.kitti_config as cnf

# Message Imports
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from navigator_msgs.msg import BoundingBox3D, Object3D, Object3DArray

# Paths to pretrained model and config
config_path = './data/perception/configs/complex_yolov4.cfg'
checkpoint_path = './data/perception/models/complex_yolov4_mse_loss.pth'

class ComplexYOLOv4Model():
    def __init__(self, device: torch.device):
        """! Initializes the node.
        @param device[torch.device]  The device the model will run on
        """
        self.img_size = 608 # Determined by the birds eye view setup
        self.device = device

        # Setup for the Complex Yolo model
        self.model = Darknet(cfgfile=config_path, 
            use_giou_loss=True) # generalized intersection over union
        self.model.load_state_dict(torch.load(checkpoint_path, 
            map_location=self.device))
        self.model.to(device=self.device)

    def preprocess(self, lidar_msg: PointCloud2):
        """! Converts ROS LiDAR message into the bird's eye view input 
            tensor for the model. 
        @param lidar_msg[PointCloud2]   The ros2 lidar message data
            formatted as a PointCloud2 message.
        @return torch.Tensor   A 608 x 608 birds eye view image of the
            point cloud data. Red = Intensity, Green = Height, Blue = 
            Density
        """
        
        # Point cloud byte stream -> np array with all fields and dtypes
        lidar_np = rnp.numpify(lidar_msg)
        # Reduces the vertical res from 128 to 64
        lidar_np = lidar_np[::2]   # matches Kitti dataset
        # Combines necessary lidar data into an array [n, n, n, n]]
        lidar_pcd = np.array([ # Avoids np array dtype issues
            lidar_np['x'].flatten(), 
            lidar_np['y'].flatten(), 
            lidar_np['z'].flatten(), 
            lidar_np['reflectivity'].flatten()])
        # Tranforms array to shape [n, (x y z r)]
        lidar_pcd = np.transpose(lidar_pcd)
        lidar_pcd[:, 3] /= 255.0 # Normalizes refl.
        
        # Removes the points outside birds eye view (bev) map
        reduced_lidar = kitti_bev_utils.removePoints(lidar_pcd, 
            cnf.boundary)
        # Turns the point cloud into a bev rgb map
        rgb_map = kitti_bev_utils.makeBVFeature(reduced_lidar, 
            cnf.DISCRETIZATION, cnf.boundary)
        # Turned into a tensor for the model
        inputs = torch.tensor(rgb_map, device=self.device).float() \
            .unsqueeze(0)
        
        return inputs

    def predict(self, inputs: torch.Tensor):
        """! Uses the tensor from the preprocess fn to return the 
            bounding boxes for the objects.
        @param inputs[torch.Tensor]  The result of the preprocess 
            function.
        @return torch.Tensor  The output of the model in the shape
            [1, boxes, x y w l classes], where the classes are Car, 
            Pedestrian, Cyclist, Van, Person_sitting.
        """
        return self.model(inputs)

    def postprocess(self, predictions: torch.Tensor, conf_thresh: int, 
            nms_thresh: int):
        """! Converts the model's predictions into a ROS2 message, and filters
            out boxes below the confidence threshold and above the intersection
            over union threshold.
        @param predictions[torch.Tensor]   The output of the model in 
            the shape [1, boxes, x y w l classes], where the classes 
            are Car, Pedestrian, Cyclist, Van, Person_sitting.
        @param conf_thresh[float]   The mininum confidence value accepted
            for bounding boxes.
        @param nms_thresh[float]   The maximum accepted intersection over
            union value for bounding boxes. 
        @return Object3DArray  A ros2 message ready to be published. 
            Before publishing, the header needs to be attached. 
        """
        # Filters boxes under conf_thresh and over nms_thresh
        detections = post_processing_v2(predictions, conf_thresh=conf_thresh, 
            nms_thresh=nms_thresh)[0]   # Returns: (x, y, w, l, im, re, object_conf, class_score, class_pred)

        # Skip if no detections made
        if detections is None:
            return None

        # Returns shape [c, c_conf, x, y, z, w, l, h, yaw]
        predictions = kitti_bev_utils.inverse_yolo_target(detections, self.img_size, cnf.boundary)

        # Turns the predictions into array of Object3D msg types
        object_3d_array = Object3DArray()
        for prediction in predictions:
            # Defines the custom ros2 msgs
            bounding_box = BoundingBox3D()
            object_3d = Object3D()

            # Need x, y, z, x_size, y_size, z_size, yaw
            bounding_box.coordinates = prediction[2:]

            # Returns the corners in the order specified by BoundingBox3D msg
            corners_3d = kitti_bev_utils.get_corners_3d(*bounding_box.coordinates)
            # Fills the Point corners array
            for i, corner_3d in enumerate(corners_3d):
                bounding_box.corners[i] = Point(x=corner_3d[0], y=corner_3d[1], z=corner_3d[2])

            object_3d.label = (int(prediction[0]) - 1) % 3 # changes label to match format of message
            object_3d.id = random.randint(0, 2**16-1)
            object_3d.confidence_score = prediction[1]
            object_3d.bounding_box = bounding_box
            
            object_3d_array.objects.append(object_3d)

        return object_3d_array