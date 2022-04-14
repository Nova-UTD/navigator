import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import ros2_numpy as rnp
from voltron_msgs.msg import Obstacle2D, Obstacle2DArray
from ament_index_python.packages import get_package_share_directory
import os
import cv2

from .tool.utils import *
from .tool.torch_utils import *
from .tool.darknet2pytorch import Darknet


COCO_TO_NOVA = {
    0:  0,  # person
    1:  1,  # bicycle
    2:  2,  # car
    3:  3,  # motorbike
    5:  4,  # bus
    7:  5   # truck
}

class DarknetInferenceNode(Node):

    def __init__(self, config_file, weights_file, names_file):
        super().__init__('darknet_inference_node')

        self.config_file = config_file
        self.weights_file = weights_file
        self.names_file = names_file

        self.model = Darknet(self.config_file)
        self.model.load_weights(self.weights_file)
        self.get_logger().info('Successfully loaded weights')
        self.model.cuda()
        
        self.class_names = load_class_names(names_file)

        self.color_image_sub = self.create_subscription(
            Image,
            '/camera_front/rgb',
            self.color_image_cb,
            10
        )

        self.obstacle_array_pub = self.create_publisher(
            Obstacle2DArray,
            '/detections/array_2d',
            10
        )

    # where the magic happens
    def color_image_cb(self, rgb_msg: Image):
        
        # preprocessing 
        color_image = rnp.numpify(rgb_msg)
        color_image = color_image[..., :3]
        sized = cv2.resize(color_image, (self.model.width, self.model.height))
        sized = cv2.cvtColor(sized, cv2.COLOR_BGR2RGB)
        
        # inference, the 2 last args are confidence threshold and NMS threshold
        boxes = do_detect(self.model, sized, 0.4, 0.4)[0]

        # filter based on class
        filtered = []
        for box in boxes:
            if box[-1] in COCO_TO_NOVA:
                filtered.append(box)

        # create msg
        obstacle_array = Obstacle2DArray()
        obstacle_array.header.frame_id = 'base_link'
        obstacle_array.header.stamp = self.get_clock().now().to_msg()

        for box in filtered:
            obstacle = Obstacle2D()
            obstacle.confidence = box[5]
            obstacle.label = COCO_TO_NOVA[box[6]]
            obstacle.bounding_box.x1 = box[0]
            obstacle.bounding_box.y1 = box[1]
            obstacle.bounding_box.x1 = box[2]
            obstacle.bounding_box.y1 = box[3]    
            obstacle_array.obstacles.append(obstacle)
   
        # publish 2d obstacles
        self.obstacle_array_pub.publish(obstacle_array)

        
def main(args=None):
    rclpy.init(args=args)

    share_dir = os.path.dirname(get_package_share_directory('darknet_inference'))
    names_file = os.path.join(share_dir, 'names/coco.names')
    config_file = os.path.join(share_dir, 'cfg/yolov4.cfg')
    weights_file = os.path.join(share_dir, 'weights/yolov4.weights')

    inference_node = DarknetInferenceNode(config_file, weights_file, names_file)

    rclpy.spin(inference_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()