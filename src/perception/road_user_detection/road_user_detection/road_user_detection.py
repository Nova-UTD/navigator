"""
Package: road_user_detection
File: road_user_detection.py
Author: Pranav Boyapati
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from navigator_msgs.msg import RoadUser
from navigator_msgs.msg import RoadUserDetections
import cv2
from ultralytics import YOLO
import numpy
import base64


class RoadUserDetection(Node):
    def __init__(self):
        super().__init__('road_user_detection')

        #initialize the classification model
        model_path = '/navigator_binaries/road_user_detector.pt'
        try:
            self.get_logger().info(f"Loading YOLO model from: {model_path}")
            self.model = YOLO(model_path)
            self.get_logger().info("YOLO model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {e}")
            raise e

        #create subscriptions
        self.camera_sub = self.create_subscription(Image, '/cameras/camera0', self.image_callback, 10)

        #create variables to store subscription info
        self.bridge = CvBridge()
        self.image = None

        #create publisher
        self.road_user_publisher = self.create_publisher(RoadUserDetections, '/road_users/detections', 10)


    #callbacks for subscriptions
    def image_callback(self, msg : Image):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='8UC3')
        self.make_detections()

    
    #main control function
    def make_detections(self):
        try:
            #gets prediction
            results = self.model(self.image)

            #create message and publish
            boxes = results[0].boxes.xyxy.tolist()      #top left corner of box to bottom right corner of box
            classes = results[0].boxes.cls.tolist()
            confidences = results[0].boxes.conf.tolist()
                
            all_road_users_msg = RoadUserDetections()
            all_road_users_msg.header.stamp = self.get_clock().now().to_msg()
            all_road_users_msg.header.frame_id = "camera_frame"

            for box, cls, conf in zip(boxes, classes, confidences):
                x1, y1, x2, y2 = box
                width = x2 - x1
                height = y2 - y1

                road_user_msg = RoadUser()
                road_user_msg.x = float(x1)
                road_user_msg.y = float(y1)
                road_user_msg.width = float(width)
                road_user_msg.height = float(height)
                road_user_msg.confidence = float(conf)
                road_user_msg.detection = str(results[0].names[int(cls)])

                all_road_users_msg.all_road_users.append(road_user_msg)
            
            self.road_user_publisher.publish(all_road_users_msg)
        except Exception as e:
            self.get_logger().error(f"Error during detection: {e}")
            return


def main(args=None):
    rclpy.init(args=args)
    road_user_detection = RoadUserDetection()
    executor = MultiThreadedExecutor()
    executor.add_node(road_user_detection)
    executor.spin()
    road_user_detection.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()
