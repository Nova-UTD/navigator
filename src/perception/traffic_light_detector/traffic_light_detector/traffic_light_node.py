'''
Package: traffic_light_detector
   File: traffic_light_node.py
 Author: Neil Agrawal

Node to filter and process raw camera footage and classify traffic lights

This node outputs 1 for green, 2 for yellow, and 3 for red.
For more information, check out the traffic_light_detector.md.
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigator_msgs.msg import TrafficLightDetection, TrafficLight

import torch
import numpy as np
import torchvision.transforms as transforms
from torchvision.models.detection import fasterrcnn_resnet50_fpn
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.ops import nms

class TrafficLightDetectorNode(Node):
    def __init__(self):
        super().__init__('traffic_light_detector_node')

        self.bridge = CvBridge()
        self.device = torch.device("cuda")
        
        # Load the pre-trained Faster R-CNN model
        self.model = fasterrcnn_resnet50_fpn(pretrained=True)
        
        # Modify the model for custom classification task (traffic lights)
        num_classes = 4  # Including background class
        in_features = self.model.roi_heads.box_predictor.cls_score.in_features
        self.model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)
        
        # Load the trained weights for fine-tuning
        model_weights_path = 'data/perception/models/fasterrcnn_resnet50_fpn.pth'
        self.model.load_state_dict(torch.load(model_weights_path, map_location=torch.device("cuda")))
        
        # Set the model to evaluation mode
        self.model.eval()

        self.subscription = self.create_subscription(
            Image, '/cameras/camera0', self.image_callback, 10)

        self.publisher = self.create_publisher(
            TrafficLightDetection, '/traffic_light/detections', 10)

    def image_callback(self, msg):
        image_data = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')

        # Run traffic light detection
        traffic_light_detections = self.detect_traffic_lights(image_data)

        # Publish traffic light detections
        self.publisher.publish(traffic_light_detections)

    def detect_traffic_lights(self, image_data):
        # Preprocess image
        image_tensor = self.preprocess_image(image_data)
        # print("Device: " + self.device)
        # Run inference
        with torch.no_grad():
            image_tensor = self.preprocess_image(image_data)
            image_tensor = image_tensor.to(torch.float32) 
            predictions = self.model(image_tensor)

        # Post-process predictions
        boxes, scores, labels = self.postprocess_predictions(predictions)

        # Prepare traffic light detections message
        traffic_light_detections_msg = TrafficLightDetection()
        traffic_light_detections_msg.header.stamp = self.get_clock().now().to_msg()
        traffic_light_detections_msg.header.frame_id = "camera_frame"

        # Populate traffic light detections message with detected traffic lights
        for box, score, label in zip(boxes, scores, labels):
            traffic_light = TrafficLight()
            # print("Stuff: "+ box[0])
            traffic_light.x = float(box[0])
            traffic_light.y = float(box[1])
            traffic_light.width = float(box[2] - box[0])
            traffic_light.height = float(box[3] - box[1])
            traffic_light.label = int(label)
            traffic_light.score = float(score)
            traffic_light_detections_msg.traffic_lights.append(traffic_light)

        return traffic_light_detections_msg

    def preprocess_image(self, image_data):
        # Convert the CV image to a PyTorch tensor
        transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),  # Assuming you want to resize the image to a fixed size
            transforms.ToTensor(),
        ])
        image_tensor = transform(image_data)
        # Add batch dimension (assuming you're working with a single image)
        image_tensor = image_tensor.unsqueeze(0)
        # Move the tensor to the appropriate device
        # image_tensor = image_tensor.to(torch.device("cuda"))
        return image_tensor

    def postprocess_predictions(self, predictions):
        boxes = predictions[0]['boxes'].cpu().numpy()
        scores = predictions[0]['scores'].cpu().numpy()
        labels = predictions[0]['labels'].cpu().numpy()
        return boxes, scores, labels

def main(args=None):
    rclpy.init(args=args)
    traffic_light_detector_node = TrafficLightDetectorNode()
    rclpy.spin(traffic_light_detector_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    rclpy.shutdown()
