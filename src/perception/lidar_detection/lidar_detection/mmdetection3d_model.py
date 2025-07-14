"""
Package:   lidar_detection
Filename:  mmdetection3d_model.py
Author:    Gueren Sanford, Ragib Arnab
Email:     guerensanford@gmail.com
Copyright: 2021, Nova UTD
License:   MIT License
Use the MMDetection3D class to make object detections with LiDAR data in the format 
[x,y,z,intensity]. We use the model MMDetection3D with PointPillars to make the 
predictions. With the predictions, feed it into the postprocess to filter out and 
format the results.
"""

# Python Imports
import random
import numpy as np
import ros2_numpy as rnp

# Model Imports
from mmdet3d.apis import init_model, inference_detector
from mmdet3d.structures.det3d_data_sample import Det3DDataSample
from mmcv.ops import nms_rotated

# Message Imports
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from navigator_msgs.msg import BoundingBox3D, Object3D, Object3DArray

# dict of class/label association
CLASS2LABEL = { 'Pedestrian': 0, 'Cyclist': 1, 'Car': 2 }

class MMDetection3DModel():
    def __init__(self, config_path: str, checkpoint_path: str, device: str):
        """! Initializes the node.
        @param config_path[str]  The config file for MMDetection3D, must use
            the config for the specific checkpoint.
        @param checkpoint_path[str]  The checkpoint file for MMDetection3D.
            Check their api for the list of checkpoint files.
        @param device[str]  The device the model will run on.
        """

        # Declare model attributes
        self.translate_height = -1.5
        self.device = device

        # Setup for the Complex Yolo model
        self.model = init_model(config=config_path, checkpoint=checkpoint_path, device=device)

    def preprocess(self, lidar_msg: PointCloud2):
        """! Converts ROS LiDAR message into a numpy for the model.
        @param lidar_msg[PointCloud2]   The ros2 lidar message data
            formatted as a PointCloud2 message.
        @return numpy.ndarray   An array of point cloud data in the shape
            [n, (x, y, z, refl)]
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
        lidar_pcd[:, 2] += self.translate_height
        
        return lidar_pcd

    def predict(self, inputs: np.ndarray):
        """! Uses the tensor from the preprocess fn to return the 
            bounding boxes for the objects.
        @param inputs[Det3DDataSample]  The result of the preprocess 
            function.
        @return Det3DDataSample  You will see a list of Det3DDataSample, 
            and the predictions are in the pred_instances_3d, indicating 
            the detected bounding boxes, labels, and scores. The classes 
            are Car, Pedestrian, Cyclist, Van, Person_sitting.
        """
        # Outputs predictions in format of Det3DDataSample class
        predictions, _ = inference_detector(model=self.model, pcds=inputs)

        return predictions

    def postprocess(self, predictions: Det3DDataSample, conf_thresh: int,  
            nms_thresh: int):
        """! Converts the model's predictions into a ROS2 message, and filters
            out boxes below the confidence threshold and above the intersection
            over union threshold.
        @param predictions[Det3DDataSample]   The output from the ppredict 
            function.
        @param conf_thresh[float]   The mininum confidence value accepted
            for bounding boxes.
        @param nms_thresh[float]   The maximum accepted intersection over
            union value for bounding boxes. 
        @return Object3DArray  A ros2 message ready to be published. 
            Before publishing, the header needs to be attached. 
        """

        bounding_boxes = predictions.pred_instances_3d.bboxes_3d
        bbox_corners = predictions.pred_instances_3d.bboxes_3d.corners
        labels = predictions.pred_instances_3d.labels_3d
        scores = predictions.pred_instances_3d.scores_3d
        detections = [bounding_boxes.tensor.double(), bbox_corners, labels, scores]

        # Filters out scores less than the conficdence threshold
        indices = scores >= conf_thresh
        detections = [tensor[indices] for tensor in detections]

        # Skip if no detections made
        if len(detections[0]) == 0:
            return None

        # Filters out boxes overlapping above nms threshold
        bounding_boxes2d = detections[0][:,[0,1,3,4,6]] # [center_x, center_y, width, height, rotation]
        _, indices = nms_rotated(bounding_boxes2d, detections[3], iou_threshold=nms_thresh)
        detections = [tensor[indices] for tensor in detections]

        # Turns the detections into array of Object3D msg types
        object_3d_array = Object3DArray()
        for bounding_box, corners, label, score in zip(*detections):
            # Defines the custom ros2 msgs
            bounding_box_3d = BoundingBox3D()
            object_3d = Object3D()

            # Shifts height of boxe down 
            bounding_box[2] -= self.translate_height
            corners[:,2] -= self.translate_height

            # Coordinates needs x, y, z, x_size, y_size, z_size, yaw
            bounding_box_3d.coordinates = bounding_box.double().cpu().numpy()

            # Converts detection corners into BoundingBox3D msg format
            index = 3
            for corner in corners:
                bounding_box_3d.corners[index % 8] = Point(
                    x=float(corner[0]), y=float(corner[1]), z=float(corner[2]))
                index -= 1

            object_3d.label = int(label)
            object_3d.confidence_score = float(score)
            object_3d.id = random.randint(0, 2**16-1)
            object_3d.bounding_box = bounding_box_3d

            object_3d_array.objects.append(object_3d)

        return object_3d_array