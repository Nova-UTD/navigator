import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
import message_filters
import ros2_numpy as rnp


import numpy as np
import cv2

import os
from ament_index_python.packages import get_package_share_directory
import time

from .DDRNet_23 import get_seg_model
import torch


device = 'cuda' if torch.cuda.is_available() else 'cpu'

camera_fov = 90
image_size_x = 1280 // 8
image_size_y = 720 // 8
focal_length = image_size_x // (2*np.tan(camera_fov * np.pi / 360))
center_x = image_size_x // 2
center_y = image_size_y // 2

intrinsic_matrix = np.array([
    [focal_length, 0, center_x, 0],
    [0, focal_length, center_y, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

extrinsic_matrix = np.array([
    [0, -1, 0, 0],
    [0, 0, -1, 2],
    [1, 0, 0, -2],
    [0, 0, 0, 1]
])

projection_matrix = np.matmul(intrinsic_matrix, extrinsic_matrix)
inv_proj = np.linalg.inv(projection_matrix)


class DDRNetInferenceNode(Node):

    def __init__(self, weights_file):
        super().__init__('ddrnet_inference_node')

        self.model = self.load_model(weights_file)
        self.get_logger().info('Successfully loaded model')

        self.mean = [0.485, 0.456, 0.406]
        self.std = [0.229, 0.224, 0.225]

        rgb_sub = message_filters.Subscriber(self, Image, "/camera_front/rgb")
        depth_sub = message_filters.Subscriber(
            self, Image, "/camera_front/depth")
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], 1, 0.5)
        self.sync.registerCallback(self.rgbd_cb)
        self.road_pcd_pub = self.create_publisher(
            PointCloud2,
            '/segmentation/road_pcd',
            10
        )

    def load_model(self, weights_file):
        model = get_seg_model(cfg=None)
        model_dict = model.state_dict()
        pretrained_dict = torch.load(weights_file, map_location=device)
        pretrained_dict = {
            k[6:]: v for k, v in pretrained_dict.items() if k[6:] in model_dict.keys()}
        model_dict.update(pretrained_dict)
        model.load_state_dict(model_dict)
        model.to(device)
        model.eval()
        return model

    def decode_segmap(self, image, nc=19):
        label_colors = np.array([(128, 0, 128),  # 0=road
                                 # 1=sidewalk, 2=building, 3=wall, 4=fence, 5=poles
                                 (192, 0, 192), (64, 64, 64), (128, 64,
                                                               0), (128, 128, 0), (128, 128, 128),
                                 # 6=traffic light, 7=traffic sign, 8=vegetation, 9=ground, 10=sky
                                 (128, 128, 0), (192, 192, 0), (0,
                                                                128, 0), (0, 192, 0), (0, 192, 255),
                                 # 11=person, 12=rider, 13=car, 14=truck, 15=bus
                                 (192, 0, 0), (255, 0, 0), (0, 0,
                                                            192), (0, 0, 128), (0, 0, 64),
                                 # 16=on rails, 17=motorcycle, 18=bicycle
                                 (0, 64, 64), (128, 64, 0), (128, 0, 0)])
        r = np.zeros_like(image).astype(np.uint8)
        g = np.zeros_like(image).astype(np.uint8)
        b = np.zeros_like(image).astype(np.uint8)
        for l in range(0, nc):
            idx = image == l
            r[idx] = label_colors[l, 0]
            g[idx] = label_colors[l, 1]
            b[idx] = label_colors[l, 2]
        rgb = np.stack([r, g, b], axis=2)
        return rgb

    def rgbd_cb(self, rgb_msg: Image, depth_msg: Image):
        color_image = rnp.numpify(rgb_msg)[..., :3]
        depth_image = rnp.numpify(depth_msg)

        assert color_image.shape[:-1] == depth_image.shape

        # pre-processing
        # double the image size so downsampling factor is 4 instead of 8 after inference
        # image = cv2.resize(
        #     color_image, (color_image.shape[1]*2, color_image.shape[0]*2), interpolation=cv2.INTER_LINEAR)
        image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        image = image.astype(np.float32)
        image = image / 255.0
        image -= self.mean
        image /= self.std
        tensor = torch.from_numpy(image).permute(
            2, 0, 1).unsqueeze(0).to(device)

        # inference
        t1 = time.time()
        seg_map = self.model(tensor)
        t2 = time.time()
        # self.get_logger().info(f'inference fps: {1/(t2-t1)}')

        # post-processing
        seg_map = torch.argmax(seg_map.squeeze(), dim=0).detach().cpu().numpy()
        road_image = np.zeros(shape=seg_map.shape, dtype=np.uint8)
        # for all classified pixels that are road (0), mark them as white (255)
        road_image[seg_map == 0] = 255

        # project image into 3-D space, don't worry about details
        xx, yy = np.meshgrid(np.arange(0, image_size_x),
                             np.arange(0, image_size_y))
        depth_image = cv2.resize(
            depth_image, (image_size_x, image_size_y), interpolation=cv2.INTER_AREA)
        pixel_array = np.stack([xx, yy, np.ones_like(
            xx), 1/(depth_image+1e-3)]).reshape(4, -1)
        points_array = np.matmul(inv_proj, pixel_array) * depth_image.flatten()
        points_array = np.transpose(
            points_array[:-1, :], (1, 0)).astype(np.float32)

        # get the points that corresponds to roads and perform some filtering
        road_pcd = points_array[road_image.flatten() == 255]
        # remove points that are above 0.2 m on z-axis
        # road_pcd = road_pcd[road_pcd[:, -1] < 0.2]

        # turn into pointcloud msg
        road_pcd.dtype = [('x', np.float32), ('y', np.float32),
                          ('z', np.float32)]  # specify fields
        road_pcd_msg = rnp.msgify(PointCloud2, road_pcd)
        road_pcd_msg.header.frame_id = 'base_link'
        road_pcd_msg.header.stamp = self.get_clock().now().to_msg()
        self.road_pcd_pub.publish(road_pcd_msg)

        # road_image_msg = rnp.msgify(Image, seg_map)


def main(args=None):
    rclpy.init(args=args)

    share_dir = os.path.dirname(
        get_package_share_directory('ddrnet_inference'))
    weights_file = os.path.join(share_dir, 'weights/best_val.pth')

    inference_node = DDRNetInferenceNode(weights_file)

    rclpy.spin(inference_node)

    inference_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
