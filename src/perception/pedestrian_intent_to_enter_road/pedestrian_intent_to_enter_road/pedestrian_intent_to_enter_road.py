"""
Package: pedestrian_intent_to_enter_road
File: pedestrian_intent_to_enter_road.py
Author: Pranav Boyapati
"""

# import libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from navigator_msgs.msg import PedestrianInfoDetections
from navigator_msgs.msg import PedestrianInfo
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from mmpose.apis import MMPoseInferencer

class PedestrianIntentToEnterRoad(Node):
    def __init__(self):
        super().__init__('pedestrian_intent_to_enter_road')

        # create subscriber and publisher
        self.camera_subscription = self.create_subscription(Image, '/cameras/camera0', self.image_callback, 10)
        self.pedestrian_publisher = self.create_publisher(PedestrianInfoDetections, '/pedestrians', 10)

        # define variables used throughout the node
        self.bridge = CvBridge()
        self.image = None
        self.detection_model = YOLO("/navigator/data/perception/models/pedestrian_detection_model.pt")
        self.inferencer = MMPoseInferencer('human')


    def image_callback(self, msg : Image):
        # convert the image message into a jpg formatted image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") 
        status, temp_image = cv2.imencode(".jpg", cv_image) 
        self.image = cv2.imdecode(temp_image, cv2.IMREAD_COLOR)

        # call the detect_pedestrians function, which performs all of the detections and filtering for this node
        self.detect_pedestrians()


    def detect_pedestrians(self):
        # perform object detection for pedestrians on the input image
        results = self.detection_model(self.image)

        # instantiate an array for holding all detections
        pedestrian_detections = []

        # this executes for each pedestrian detected by the model
        for result in results:
            # get the location and dimensions of the pedestrian bounding box
            xywh = result.boxes.xywh
            center_x, center_y, width, height = xywh

            direction_facing = ""

            xyxy = result.boxes.xyxy
            TLx, TLy, BRx, BRy = xyxy

            image_to_crop = cv2.imread(self.image)
            cropped_img = image_to_crop[TLy:BRy, TLx:BRx].copy()
            image_of_person = cv2.imdecode(cropped_img, cv2.IMREAD_COLOR)

            result_generator = self.inferencer(image_of_person, show=False)
            pose_estimation_result = next(result_generator)

            nose_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][0]
            left_eye_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][1]
            right_eye_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][2]

            if ((nose_keypoint[0] > left_eye_keypoint[0]) and (nose_keypoint[0] > right_eye_keypoint[0])):
                direction_facing = "RIGHT"
            elif ((nose_keypoint[0] < left_eye_keypoint[0]) and (nose_keypoint[0] < right_eye_keypoint[0])):
                direction_facing = "LEFT"
            else:
                continue

            height, width, channels = self.image.shape
            midpoint_x = width / 2

            if (((direction_facing == "RIGHT") and (center_x < midpoint_x)) or ((direction_facing == "LEFT") and (center_x > midpoint_x))):
                # for pedestrians facing the road, determine the distance between them and the road
                horizontal_dist = self.calculate_horizontal_distance()

                # create a PedestrianInfo message object for the pedestrian and append to the detections array
                pedestrian_object = PedestrianInfo()

                pedestrian_object.x = center_x
                pedestrian_object.y = center_y
                pedestrian_object.width = width
                pedestrian_object.height = height
                pedestrian_object.distance = horizontal_dist

                pedestrian_detections.append(pedestrian_object)
            else:
                continue

        # create and publish a PedestrianInfoDetections message
        detections_msg = PedestrianInfoDetections()

        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = "camera_frame"
        detections_msg.pedestrians = pedestrian_detections

        self.pedestrian_publisher.publish(detections_msg)


    def calculate_horizontal_distance(self):
        # TODO: determine the horizontal distance between the pedestrian and the edge of the road


def main(args=None):
    rclpy.init(args=args)
    pedestrian_intent_to_enter_road = PedestrianIntentToEnterRoad()
    rclpy.spin(pedestrian_intent_to_enter_road)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()
