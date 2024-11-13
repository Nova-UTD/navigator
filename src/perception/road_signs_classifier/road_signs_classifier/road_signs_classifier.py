"""
Package: road_signs_classifier
File: road_signs_classifier.py
Author: Pranav Boyapati
"""

import rclpy
import base64
import os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigator_msgs.msg import RoadSigns
from navigator_msgs.msg import RoadSignsDetection
import cv2
from inference_sdk import InferenceHTTPClient


class RoadSignsClassifier(Node):
    def __init__(self):
        super().__init__('road_signs_classifier')

        #instantiate model client
        self.CLIENT = InferenceHTTPClient(api_url="http://localhost:6060", api_key="BmqYjCBXZD1iPIyq09sG")

        #create subscriptions
        self.camera_sub = self.create_subscription(Image, '/cameras/camera0', self.image_callback, 10)

        #create variables to store subscription info
        self.bridge = CvBridge()
        self.image = None

        #create publisher
        self.road_signs_publisher = self.create_publisher(RoadSignsDetection, '/road_signs/detections', 10)


    #callbacks for subscriptions
    def image_callback(self, msg : Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") 
        status, self.image = cv2.imencode(".jpeg", cv_image) 
        self.classify_sign()

    
    #main control function
    def classify_sign(self):
        #gets prediction
        img_base64 = base64.b64encode(self.image).decode('utf-8')
        result = self.CLIENT.infer(img_base64, model_id="us-road-signs/71")

        #create message and publish
        if (len(result["predictions"]) > 0):

            road_signs_detection_msg = RoadSignsDetection()
            road_signs_detection_msg.header.stamp = self.get_clock().now().to_msg()
            road_signs_detection_msg.header.frame_id = "camera_frame"
            
            road_sign = RoadSigns()
            road_sign.x = result["predictions"][0]['x']
            road_sign.y = result["predictions"][0]['y']
            road_sign.width = result["predictions"][0]['width']
            road_sign.height = result["predictions"][0]['height']
            road_sign.label = result["predictions"][0]['class']
            road_sign.confidence = result["predictions"][0]['confidence']

            road_signs_detection_msg.road_signs.append(road_sign)

            self.road_signs_publisher.publish(road_signs_detection_msg)


def main(args=None):
    rclpy.init(args=args)
    road_signs_classification = RoadSignsClassifier()
    rclpy.spin(road_signs_classification)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()