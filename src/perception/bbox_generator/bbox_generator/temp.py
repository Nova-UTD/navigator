import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from voltron_msgs.msg import Obstacle2D, Obstacle2DArray, Obstacle3D, Obstacle3DArray
from geometry_msgs.msg import Point
import message_filters
import ros2_numpy as rnp
from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener

import numpy as np
from scipy.spatial.transform import Rotation as R
from sklearn.cluster import DBSCAN
import cv2


CAMERA_FOV = 90
IMAGE_SIZE_X = 800
IMAGE_SIZE_Y = 600
FOCAL_LENGTH = IMAGE_SIZE_X // (2*np.tan(CAMERA_FOV * np.pi / 360))
CENTER_X = IMAGE_SIZE_X // 2
CENTER_Y = IMAGE_SIZE_Y // 2

Z_LIMIT = 30

INTRINSIC_MATRIX = np.array([
   [FOCAL_LENGTH, 0, CENTER_X, 0],
    [0, FOCAL_LENGTH, CENTER_Y, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

EXTRINSIC_MATRIX = np.array([
    [0, -1, 0, 0],
    [0, 0, -1,2],
    [1, 0, 0, -2],
    [0, 0, 0, 1]
])

PROJECTION_MATRIX = np.matmul(INTRINSIC_MATRIX, EXTRINSIC_MATRIX)
INV_PROJ = np.linalg.inv(PROJECTION_MATRIX)

CLASS_EPS = {
    0:  0.50,   # person
    1:  0.50,   # bicycle
    2:  0.8,    # car
    3:  0.60,   # motorbike
    4:  1.25,   # bus
    5:  1.0,    # truck
}


class BBoxGeneratorNode(Node):

    def __init__(self):
        super().__init__('bbox_gen_node')

        depth_sub = message_filters.Subscriber(self, Image, '/camera_front/depth')
        obstacles_2d_sub = message_filters.Subscriber(self, Obstacle2DArray, '/detections/array_2d')
        self.sync = message_filters.ApproximateTimeSynchronizer([depth_sub, obstacles_2d_sub], 1, 0.5)
        self.sync.registerCallback(self.bbox_gen_cb)

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/point_cloud/cloud',
            self.lidar_cb,
            10
        )

        self.pcd_buffer = None

        self.obstacles_3d_pub = self.create_publisher(
            Obstacle3DArray,
            '/detections/array_3d',
            10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar_tf = TransformStamped()


    def bbox_gen_cb(self, depth_msg: Image, obstacles_2d: Obstacle2DArray):
        depth_image = rnp.numpify(depth_msg)
        route_lidar = []
        obstacles_3d_array = Obstacle3DArray()
        for obstacle in obstacles_2d.obstacles:
            # get the bounding box in pixels
            x1 = int(obstacle.bounding_box.x1 * IMAGE_SIZE_X)
            y1 = int(obstacle.bounding_box.y1 * IMAGE_SIZE_Y)
            x2 = int(obstacle.bounding_box.x2 * IMAGE_SIZE_X)
            y2 = int(obstacle.bounding_box.y2 * IMAGE_SIZE_Y)
            confidence = obstacle.confidence
            class_label = obstacle.label

            # get the depth of center pixel of bounding box
            center_depth = depth_image[(y2-y1)//2, (x2-x1)//2]

            # if the obstacle is not within limits of depth camera,
            # pass the box generation to lidar-based method
            if (center_depth >= Z_LIMIT):
                route_lidar.append(obstacle)
                continue


            # project 2-d bounding box section of image into 3-D
            xx, yy = np.meshgrid(np.arange(x1, x2), np.arange(y1, y2))
            depth_section = depth_image[y1:y2, x1:x2]
            print("coordinates: ", x1, x2, y1, y2)
            print("shapes before fix: ", xx.shape, yy.shape, depth_section.shape)
            if xx.shape != depth_section.shape:
                print("I was here")
                min_y = min(xx.shape[0], depth_image.shape[0])
                min_x = min(xx.shape[1], depth_image.shape[1])
                xx = xx[:min_y, :min_x]
                yy = yy[:min_y, :min_x]
                depth_section = depth_section[:min_y, :min_x]
            print("shapes after fix: ", xx.shape, yy.shape, depth_section.shape)
            pixel_array = np.stack([xx, yy, np.ones_like(xx), 1/(depth_section+1e-2)]).reshape(4,-1)
            points_array = np.matmul(INV_PROJ, pixel_array) * depth_section.flatten()
            points_array = np.transpose(points_array[:-1, :], (1, 0)).astype(np.float32)

            # clip out the ground, anything below 0.2 m in z
            points_array = points_array[points_array[:, -1] >= 0.2]

            # bail out if everything is clipped out
            print("shape of points_array: ", points_array.shape)
            if points_array.size == 0:
                return 

            # select a random number of points uniformly
            num_points = points_array.shape[0]
            num_select_points = 300
            if num_select_points < num_points:
                random_indices = np.random.choice(num_points, num_select_points, replace=False)
                points_array = points_array[random_indices]

            # get 8 box corners
            bbox_3d = self.generate_bbox(points_array, class_label)
            if bbox_3d is None:
                continue

            # create obstacle msg
            obstacle_3d = Obstacle3D()
            obstacle_3d.confidence = confidence
            obstacle_3d.label = class_label
            for i in range(8):
                point = Point()
                point.x = bbox_3d[i][0]
                point.y = bbox_3d[i][1]
                point.z = bbox_3d[i][2]
                obstacle_3d.bounding_box.corners[i] = point
            
            obstacles_3d_array.obstacles.append(obstacle_3d)

        obstacles_3d_array.header.frame_id = 'base_link'
        obstacles_3d_array.header.stamp = self.get_clock().now().to_msg()
        self.obstacles_3d_pub.publish(obstacles_3d_array)
        
            


    def generate_bbox(self, points_array: np.ndarray, class_label):
        X = points_array[:, :-1]
        num_points = X.shape[0]

        # cluster
        db = DBSCAN(eps=CLASS_EPS[class_label], min_samples=10).fit(X)
        labels = db.labels_

        # this section just gets the largest cluster, ignores -1s
        unique, counts = np.unique(labels, return_counts=True)
        if -1 in unique:
            idx = unique > -1
            unique, counts = unique[idx], counts[idx]
        try:    
            label_most = unique[np.argmax(counts)]

        # if accessing empty sequence it means nothing could be clustered
        except ValueError:
            self.get_logger().info('failed to cluster object')
            return
            
        points_mask = labels == label_most
        cluster_points_xy = X[points_mask].astype(np.float32)

        # calculate the best fit min xy bbox
        ret = cv2.minAreaRect(cluster_points_xy)
        bbox_2d = cv2.boxPoints(ret)

        # get the height of the box
        height = np.max((points_array[points_mask])[:, -1])
        
        # create 3-D bbox
        bbox_lower = np.concatenate([bbox_2d, np.zeros((4,1))], axis=1)
        bbox_upper = np.concatenate([bbox_2d, np.full((4,1), height)], axis=1)
        bbox_3d = np.vstack([bbox_lower, bbox_upper]) 

        return bbox_3d


    def lidar_cb(self, lidar_msg: PointCloud2):
        pcd = rnp.numpify(lidar_msg)
        if lidar_msg.header.frame_id == 'lidar_front':
            self.get_lidar_tf('lidar_front')
            if self.tf_buffer.can_transform('base_link', 'lidar_front', self.get_clock().now()):
                # TO-DO
                return
            else:
                self.get_logger().info('Cannot transform lidar_front to base_link')
                return
        elif lidar_msg.header.frame_id != 'base_link':
            # something is wronge with frame_id
            self.get_logger().info('Received lidar scan with invalid frame')
            return
        self.pcd_buffer = pcd


    def get_lidar_tf(self, lidar_frame: str):
        try:
            self.lidar_tf = self.tf_buffer.lookup_transform(
                'base_link',
                lidar_frame,
                rclpy.time.Time(seconds=0, nanoseconds=0)
            )

        except TransformException as ex:
            self.get_logger().info(f'Could not transform base_link to {lidar_frame}: {ex}')
            return
    

         
def main(args=None):
    rclpy.init(args=args)

    bbox_gen_node = BBoxGeneratorNode()

    rclpy.spin(bbox_gen_node)

    bbox_gen_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()