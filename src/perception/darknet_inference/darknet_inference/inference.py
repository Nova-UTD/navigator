import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import ros2_numpy as rnp
from voltron_msgs.msg import Obstacle2D, Obstacle2DArray
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

# use the last gpu if more than one, else use the first one
cuda = torch.device(f'cuda:{torch.cuda.device_count()-1}')


class DarknetInferenceNode(Node):

    def __init__(self):
        super().__init__('darknet_inference_node')

        self.declare_parameter(
            'config_file', 'data/perception/yolov4.cfg')
        self.declare_parameter(
            'weights_file', 'data/perception/yolov4.weights')
        self.declare_parameter(
            'names_file', 'data/perception/coco.names')

        self.config_file = self.get_parameter(
            'config_file').get_parameter_value().string_value
        self.weights_file = self.get_parameter(
            'weights_file').get_parameter_value().string_value
        self.names_file = self.get_parameter(
            'names_file').get_parameter_value().string_value

        self.model = Darknet(self.config_file)
        self.model.load_weights(self.weights_file)
        self.get_logger().info('Successfully loaded weights')
        self.model.to(cuda)

        self.class_names = load_class_names(self.names_file)

        self.color_image_sub = self.create_subscription(
            Image,
            '/color_image',
            self.color_image_cb,
            10
        )

        self.obstacle_array_pub = self.create_publisher(
            Obstacle2DArray,
            '/obstacle_array_2d',
            10
        )

        self.declare_parameter('publish_labeled_image', False)
        self.pub_image = self.get_parameter(
            'publish_labeled_image').get_parameter_value().bool_value
        if self.pub_image:
            self.labeled_image_pub = self.create_publisher(
                Image,
                '/labeled_image',
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
        boxes = do_detect(self.model, sized, 0.4, 0.4, cuda)[0]

        # filter based on class
        filtered = []
        for box in boxes:
            if box[-1] in COCO_TO_NOVA:
                filtered.append(box)

        # plot bounding boxes
        if self.pub_image:
            labeled_image = cv2.resize(color_image, (640, 360))
            labeled_image = plot_boxes_cv2(
                labeled_image, filtered, class_names=self.class_names)

        # create msg
        obstacle_array = Obstacle2DArray()
        for box in filtered:
            obstacle = Obstacle2D()
            obstacle.confidence = float(box[5])
            obstacle.label = COCO_TO_NOVA[box[6]]
            x1, y1, x2, y2 = np.clip(np.array(box[:4]), a_min=0.0, a_max=1.0)
            obstacle.bounding_box.x1 = x1.item()
            obstacle.bounding_box.y1 = y1.item()
            obstacle.bounding_box.x2 = x2.item()
            obstacle.bounding_box.y2 = y2.item()
            obstacle_array.obstacles.append(obstacle)

        if len(obstacle_array.obstacles) != 0:
            # publish obstacles
            obstacle_array.header.frame_id = 'base_link'
            obstacle_array.header.stamp = self.get_clock().now().to_msg()
            self.obstacle_array_pub.publish(obstacle_array)

            # publish labeled image
            if self.pub_image:
                labeled_image_msg: Image = rnp.msgify(
                    Image, labeled_image, encoding='8UC3')
                labeled_image_msg.header.stamp = self.get_clock().now().to_msg()
                labeled_image_msg.header.frame_id = 'labeled_image'
                self.labeled_image_pub.publish(labeled_image_msg)
        else:
            return


def main(args=None):
    rclpy.init(args=args)

    inference_node = DarknetInferenceNode()

    rclpy.spin(inference_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
