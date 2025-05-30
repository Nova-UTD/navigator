import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import torch
import cv2

from sam2.build_sam import build_sam2
from sam2.sam2_image_predictor import SAM2ImagePredictor

class ImageSegNode(Node):
    def __init__(self):
        super().__init__('image_seg_node')

        self.signal_image_sub = self.create_subscription(Image, "/ouster/signal_image", self.image_callback, qos_profile_sensor_data)

        self.segmentation_pub = self.create_publisher(Image, "/segmentation_mask", 10)

        self.bridge = CvBridge()

        #initialize sam 2.0
        self.device = "cuda:0"
        self.sam_checkpoint = "sam2.1_hiera_base_plus.pt"
        self.model_cfg = "/configs/sam2.1/sam2.1_hiera_b+.yaml"
        self.predictor = SAM2ImagePredictor(build_sam2(self.model_cfg, self.sam_checkpoint))

        print("Started!")

    def image_callback(self, msg):

        #ros2 to opencv format
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        print(f"Image shape: {image.shape}, Min: {np.min(image)}, Max: {np.max(image)}")
        
        #apply sam
        segmentation_mask = self.run_sam_segmentation(image)
        
        #lublish mask
        mask_msg = self.bridge.cv2_to_imgmsg(segmentation_mask.astype(np.uint8), encoding="mono8")
        mask_msg.header = msg.header
        self.segmentation_pub.publish(mask_msg)

    def run_sam_segmentation(self, image):
        self.predictor.set_image(image)
        h, w, _ = image.shape
        input_point = np.array([[w // 2, h // 2]])  #choose center of image
        input_label = np.array([1])  #input label (1)

        #generate different masks with different confidence scores, then generate confidence scores for each mask
        masks, scores, _ = self.predictor.predict(input_point, input_label, multimask_output=True)
        
        #find mask with the biggest confidence score
        best_mask = masks[np.argmax(scores)]

        #choose all mask values greater than 0.5 to true and everything to false, and convert 1 (drivable) to white (255) and 0 (non-drivable) to 0 (black)
        best_mask = (best_mask > 0.2).astype(np.uint8) * 255 
        print("test print")
        print(f"Segmentation Mask min: {np.min(best_mask)}, max: {np.max(best_mask)}") 
        
        #save image to disk (temporary)
        self.bridge.cv2.imwrite("output_image.jpg", best_mask)
        print("Saved image to disk")

        #return binary mask
        return best_mask

def main(args=None):
    rclpy.init(args=args)
    isn = ImageSegNode()
    rclpy.spin(isn)
    isn.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
