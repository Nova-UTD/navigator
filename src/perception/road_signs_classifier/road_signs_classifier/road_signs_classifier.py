"""
Package: road_signs_classifier
File: road_signs_classifier.py
Author: Pranav Boyapati
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigator_msgs import RoadSigns
from navigator_msgs import RoadSignsDetection
import cv2
from inference_sdk import InferenceHTTPClient


class road_signs_classifier(Node):
    def __init__(self):
        super().__init__('road_signs_classifier')

        #instantiate model client
        CLIENT = InferenceHTTPClient(api_url="http://localhost:6060", api_key="BmqYjCBXZD1iPIyq09sG")

        #Create timer for calling navigate_intersection function
        self.create_timer(0.001, self.classify_sign)

        #create subscriptions
        self.camera_sub = self.create_subscription(Image, '/cameras/cameraX', self.image_callback, 10)

        #create variables to store subscription info
        self.image = None
        self.bridge = CvBridge()

        #create publisher
        self.road_signs_publisher = self.create_publisher(RoadSignsDetection, '/road_signs/detections', 10)


    #callbacks for subscriptions
    def image_callback(self, msg : Image):
        temp_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
        self.image = cv.imencode('.jpg', temp_image)


    #main control function
    def classify_sign(self):
        #gets prediction
        result = CLIENT.infer(self.image, model_id="us-road-signs/71")

        #create message and publish
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
    road_signs_classifier = road_signs_classifier()
    rclpy.spin(road_signs_classifier)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()