"""
Package: pedestrian_intent_to_enter_road
File: pedestrian_intent_to_enter_road.py
Author: Pranav Boyapati
"""

# import libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
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
        self.binary_mask_subscription = self.create_subscription(Image, '/segmentation_mask', self.binary_mask_callback, 10)
        self.pedestrian_publisher = self.create_publisher(PedestrianInfoDetections, '/pedestrians', 10)

        # define variables used throughout the node
        self.bridge = CvBridge()
        self.image = None
        self.detection_model = YOLO("/navigator/data/perception/models/pedestrian_detection_model.pt")
        self.inferencer = MMPoseInferencer('human')
        self.binary_mask = None

        self.FACING_RIGHT = "RIGHT"
        self.FACING_LEFT = "LEFT"


    def image_callback(self, msg : Image):
        # convert the image message into a jpg formatted image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") 
        status, temp_image = cv2.imencode(".jpg", cv_image) 
        self.image = cv2.imdecode(temp_image, cv2.IMREAD_COLOR)

        # call the detect_pedestrians function, which performs all of the detections and filtering for this node
        self.detect_pedestrians()


    def binary_mask_callback(self, msg : Image):
        # convert the image message into a jpg formatted image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough") 
        status, temp_image = cv2.imencode(".jpg", cv_image) 
        self.binary_mask = cv2.imdecode(temp_image, cv2.IMREAD_COLOR)


    def detect_pedestrians(self):
        # perform object detection for pedestrians on the input image
        results = self.detection_model(self.image)

        # instantiate an array for holding all detections
        pedestrian_detections = []

        # this executes for each pedestrian detected by the model
        for result in results:
            for i in range(len(results[0].boxes)):
                try:
                    # get the location and dimensions of the pedestrian bounding box
                    xywh = result.boxes.xywh[i].cpu().numpy()
                    center_x, center_y, width, height = xywh

                    direction_facing = ""

                    xyxy = result.boxes.xyxy[i].cpu().numpy()
                    TLx, TLy, BRx, BRy = xyxy

                    image_of_person = self.image[int(TLy):int(BRy), int(TLx):int(BRx)].copy()

                    result_generator = self.inferencer(image_of_person, show=False)
                    pose_estimation_result = next(result_generator)

                    nose_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][0]
                    left_eye_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][1]
                    right_eye_keypoint = pose_estimation_result["predictions"][0][0]["keypoints"][2]

                    if ((nose_keypoint[0] > left_eye_keypoint[0]) and (nose_keypoint[0] > right_eye_keypoint[0])):
                        direction_facing = self.FACING_RIGHT
                    elif ((nose_keypoint[0] < left_eye_keypoint[0]) and (nose_keypoint[0] < right_eye_keypoint[0])):
                        direction_facing = self.FACING_LEFT
                    else:
                        continue

                    height, width, channels = self.image.shape
                    midpoint_x = width / 2

                    if (((direction_facing == self.FACING_RIGHT) and (center_x < midpoint_x)) or ((direction_facing == self.FACING_LEFT) and (center_x > midpoint_x))):
                        # for pedestrians facing the road, determine the distance between them and the road
                        horizontal_dist = self.calculate_horizontal_distance(bounding_box_coordinates=xyxy, direction_facing=direction_facing)

                        # create a PedestrianInfo message object for the pedestrian and append to the detections array
                        pedestrian_object = PedestrianInfo()

                        pedestrian_object.x = float(center_x)
                        pedestrian_object.y = float(center_y)
                        pedestrian_object.width = float(width)
                        pedestrian_object.height = float(height)
                        pedestrian_object.distance = float(horizontal_dist)

                        pedestrian_detections.append(pedestrian_object)
                    else:
                        continue
                except Exception as e:
                    self.get_logger().error(f"Error detecting pedestrians: {e}")

        # create and publish a PedestrianInfoDetections message
        detections_msg = PedestrianInfoDetections()

        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = "camera_frame"
        detections_msg.pedestrians = pedestrian_detections

        self.pedestrian_publisher.publish(detections_msg)


    def calculate_horizontal_distance(self, bounding_box_coordinates, direction_facing):
        num_pixels = 0
        TLx, TLy, BRx, BRy = bounding_box_coordinates

        if (self.binary_mask is not None):
            if (direction_facing == self.FACING_RIGHT):
                while (True):
                    pixel_color = self.binary_mask[BRx + num_pixels, BRy]

                    if (pixel_color == [0, 0, 0]):
                        break
                    else:
                        num_pixels += 1
            elif (direction_facing == self.FACING_LEFT):
                while (True):
                    pixel_color = self.binary_mask[TLx - num_pixels, BRy]

                    if (pixel_color == [0, 0, 0]):
                        break
                    else:
                        num_pixels += 1

            depth = 470 * 1.75 / (BRy - TLy)        # focal length * real height of person / height of person in pixels
            total_width_at_person_depth = 2 * depth * 1.0913        # 2 * depth * tan(FoV / 2)
            height, width, channels = self.image.shape
            hor_dist = num_pixels / width * total_width_at_person_depth

            return hor_dist
        else:
            return 1000000


def main(args=None):
    rclpy.init(args=args)
    pedestrian_intent_to_enter_road = PedestrianIntentToEnterRoad()
    executor = MultiThreadedExecutor()
    executor.add_node(pedestrian_intent_to_enter_road)
    executor.spin()
    pedestrian_intent_to_enter_road.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
    rclpy.shutdown()