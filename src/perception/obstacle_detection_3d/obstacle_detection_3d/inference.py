import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from voltron_msgs.msg import Obstacle2DArray, Obstacle3D, Obstacle3DArray
from voltron_msgs.msg import Landmark, LandmarkArray
from geometry_msgs.msg import Point
import ros2_numpy as rnp

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation as R

DEPTH_MAX = 35.0    # 35 just to be safe
RANGE_MAX = 60.0
SEARCH_RADIUS = 1.0

LANDMARKS = {
    6:  'FIRE_HYDRANT',
    7:  'STOP_SIGN'
}

CLASS_X = {
    0:  0.50,   # person
    1:  1.5,   # bicycle
    2:  4.5,    # car
    3:  2.0,   # motorbike
    4:  10.0,   # bus
    5:  5.0,    # truck
}


class ObstacleDetection3DNode(Node):

    def __init__(self):
        super().__init__('obstacle_detector_3d_node')

        # parameter declarations
        self.declare_parameter('image_size_x', 1280)
        self.declare_parameter('image_size_y', 720)
        self.declare_parameter('focal_x', 1280/2)
        self.declare_parameter('focal_y', 720/2)
        self.declare_parameter('center_x', 1280/2)
        self.declare_parameter('center_y', 720/2)
        self.declare_parameter('rotation_x', 0.0)
        self.declare_parameter('rotation_y', 0.0)
        self.declare_parameter('rotation_z', 0.0)
        self.declare_parameter('translation_x', 0.0)
        self.declare_parameter('translation_y', 0.0)
        self.declare_parameter('translation_z', 0.0)

        # get image resolution
        self.image_size_x = self.get_parameter('image_size_x').get_parameter_value().integer_value
        self.image_size_y = self.get_parameter('image_size_y').get_parameter_value().integer_value

        # get intrinsics
        focal_x = self.get_parameter('focal_x').get_parameter_value().double_value
        focal_y = self.get_parameter('focal_y').get_parameter_value().double_value
        center_x = self.get_parameter('center_x').get_parameter_value().double_value
        center_y = self.get_parameter('center_y').get_parameter_value().double_value

        intrinsic_matrix = np.array([
            [focal_x, 0, center_x, 0],
            [0, focal_y, center_y, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])

        # get extrinsics
        rot_x = self.get_parameter('rotation_x').get_parameter_value().double_value
        rot_y = self.get_parameter('rotation_y').get_parameter_value().double_value
        rot_z = self.get_parameter('rotation_z').get_parameter_value().double_value
        trans_x = self.get_parameter('translation_x').get_parameter_value().double_value
        trans_y = self.get_parameter('translation_y').get_parameter_value().double_value
        trans_z = self.get_parameter('translation_z').get_parameter_value().double_value

        # matrix to convert to ros coordinate frame convention
        camera_to_world_matrix = np.array([
            [0, -1, 0, 0],
            [0, 0, -1, 0],
            [1, 0, 0, 0],
            [0, 0, 0, 1]
        ])

        # matrix to transform to base link
        try:    # version issue: as_dcm was named to as_matrix
            rotation_matrix = R.from_euler('xyz', [rot_x, rot_y, rot_z], degrees=True).as_matrix()
        except AttributeError:
            rotation_matrix = R.from_euler('xyz', [rot_x, rot_y, rot_z], degrees=True).as_dcm()

        translation_vector = np.array([[trans_x], [trans_y], [trans_z]])
        bl_transform_matrix = np.concatenate([rotation_matrix, translation_vector], axis=1)
        bl_transform_matrix = np.concatenate((bl_transform_matrix,[[0,0,0,1]]), axis=0)

        extrinsic_matrix = np.matmul(camera_to_world_matrix, bl_transform_matrix)

        # calculate projection matrix and its inverse
        projection_matrix = np.matmul(intrinsic_matrix, extrinsic_matrix)
        self.inv_proj_matrix = np.linalg.inv(projection_matrix)

        self.obstacle_array_2d_cb = self.create_subscription(
            Obstacle2DArray,
            '/obstacle_array_2d',
            self.inference_3d,
            10
        )

        self.depth_image = None
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_image',
            self.depth_image_cb,
            10
        )

        self.pcd_kdtree = None
        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_fused',
            self.lidar_cb,
            10
        )

        self.obstacles_3d_pub = self.create_publisher(
            Obstacle3DArray,
            '/obstacle_array_3d',
            10
        )

        self.landmark_pub = self.create_publisher(
            LandmarkArray,
            '/landmarks',
            10
        )

    # generate 3-d bounding box from 2-d bounding boxes
    # or generate 3-d landmark positions
    def inference_3d(self, obstacles_2d: Obstacle2DArray):
        if type(self.depth_image) != np.ndarray:
            return
        else:
            depth_image = self.depth_image

        obstacles_3d_array = Obstacle3DArray()
        landmark_array = LandmarkArray()
        for index, obstacle in enumerate(obstacles_2d.obstacles):
            # get the bounding box in pixels
            x1 = int(obstacle.bounding_box.x1 * self.image_size_x)
            y1 = int(obstacle.bounding_box.y1 * self.image_size_y)
            x2 = int(obstacle.bounding_box.x2 * self.image_size_x)
            y2 = int(obstacle.bounding_box.y2 * self.image_size_y)

            # get the approx min depth of bounding box
            bbox_depth = depth_image[y1:y2, x1:x2]
            bbox_depth = bbox_depth[np.isfinite(bbox_depth)]
            if bbox_depth.size == 0:
                continue
            min_depth = np.percentile(bbox_depth, 5)
            bbox_3d = np.zeros((8, 3))

            # publish landmarks by themselves
            if obstacle.label in LANDMARKS:
                if (min_depth <= DEPTH_MAX):    # if within range of depth camera
                    center_pixel_x = (x2+x1) // 2
                    center_pixel_y = (y2+y1) // 2
                    center_pixel = np.array(
                            [center_pixel_x, center_pixel_y, 1, 1/min_depth]).reshape(4, -1)
                    center_point = np.matmul(self.inv_proj_matrix, center_pixel) * min_depth
                    landmark = Landmark()
                    landmark.center_point.x = center_point[0].item()
                    landmark.center_point.y = center_point[1].item()
                    landmark.center_point.z = center_point[2].item()
                    landmark.label = obstacle.label
                    landmark.confidence = obstacle.confidence
                    landmark_array.landmarks.append(landmark)
                continue

            # if the obstacle is further than max depth of camera
            if (min_depth >= DEPTH_MAX):
                if self.pcd_kdtree != None:  # pass the box generation to lidar-based method
                    # center of 2D bbox into 3D
                    center_pixel_x = (x2+x1) // 2
                    center_pixel_y = (y2+y1) // 2

                    largest_query = []
                    for z in np.arange(DEPTH_MAX, RANGE_MAX):
                        # project the center pixel of 2D bbox for different depth values
                        center_pixel = np.array(
                            [center_pixel_x, center_pixel_y, 1, 1/z]).reshape(4, -1)
                        proj_point = np.matmul(self.inv_proj_matrix, center_pixel) * z
                        proj_point = np.transpose(proj_point[:-1, :], (1, 0))

                        # get the nearest neighbor points to the projected center pixel within a radius
                        query_indices = self.pcd_kdtree.query_ball_point(
                            proj_point, SEARCH_RADIUS)[0]

                        # store the query that resulted in most points
                        if len(query_indices) > len(largest_query):
                            largest_query = query_indices

                    # get the min depth of the largest cluster, which should be the object
                    largest_cluster = self.pcd_kdtree.data[largest_query]
                    if largest_cluster.size == 0:
                        continue
                    #min_depth = np.percentile(largest_cluster[:, 0], 10)
                    min_depth = largest_cluster[:, 0].min()

                else:   # lidar data doesn't exist
                    continue

            # project 2-d bounding box into 3-d
            xx, yy = np.meshgrid([x1, x2], [y1, y2])
            depth_array = np.full_like(xx, min_depth, dtype=np.float32)
            pixel_array = np.stack([xx, yy, np.ones_like(
                xx), 1/(depth_array+1e-3)]).reshape(4, -1)
            bbox_3d_front = np.matmul(
                self.inv_proj_matrix, pixel_array) * depth_array.flatten()
            bbox_3d_front = np.transpose(
                bbox_3d_front[:-1, :], (1, 0)).astype(np.float32)

            # get 8 box corners in msg format
            bbox_3d_back = np.copy(bbox_3d_front)
            bbox_3d_back[:, 0] += CLASS_X[obstacle.label]
            bbox_3d[0] = bbox_3d_front[0]
            bbox_3d[1] = bbox_3d_back[0]
            bbox_3d[2] = bbox_3d_back[1]
            bbox_3d[3] = bbox_3d_front[1]
            bbox_3d[4] = bbox_3d_front[2]
            bbox_3d[5] = bbox_3d_back[2]
            bbox_3d[6] = bbox_3d_back[3]
            bbox_3d[7] = bbox_3d_front[3]

            # create obstacle msg
            obstacle_3d = Obstacle3D()
            obstacle_3d.confidence = obstacle.confidence
            obstacle_3d.label = obstacle.label
            obstacle_3d.id = index
            for i in range(8):
                point = Point()
                point.x = bbox_3d[i][0].item()
                point.y = bbox_3d[i][1].item()
                point.z = bbox_3d[i][2].item()
                obstacle_3d.bounding_box.corners[i] = point

            obstacles_3d_array.obstacles.append(obstacle_3d)

        if len(obstacles_3d_array.obstacles) != 0:
            obstacles_3d_array.header.frame_id = 'base_link'
            obstacles_3d_array.header.stamp = self.get_clock().now().to_msg()
            self.obstacles_3d_pub.publish(obstacles_3d_array)

        if len(landmark_array.landmarks) != 0:
            landmark_array.header.frame_id = 'base_link'
            landmark_array.header.stamp = self.get_clock().now().to_msg()
            self.landmark_pub.publish(landmark_array)

    # cache depth image
    def depth_image_cb(self, depth_msg: Image):
        # self.get_logger().info('got depth')
        self.depth_image = rnp.numpify(depth_msg)

    # expects transformed, fused lidar point clouds
    # filters the point cloud and caches it as a K-d Tree
    def lidar_cb(self, lidar_msg: PointCloud2):
        # self.get_logger().info('got lidar_cb')
        pcd = rnp.numpify(lidar_msg)
        if lidar_msg.header.frame_id != 'base_link':
            # something is wrong with frame_id
            self.get_logger().info('Received lidar scan with invalid frame')
            return

        # remove the floor
        pcd = pcd[pcd['z'] > 0.0]

        # get it into kdtree data format
        x, y, z = pcd['x'], pcd['y'], pcd['z']
        data = np.stack([x, y, z], axis=-1).reshape(-1, 3)

        # store as kdtree
        self.pcd_kdtree = cKDTree(data)


def main(args=None):
    rclpy.init(args=args)

    obstacle_detection_3d_node = ObstacleDetection3DNode()

    rclpy.spin(obstacle_detection_3d_node)

    obstacle_detection_3d_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
