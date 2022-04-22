import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from voltron_msgs.msg import Obstacle2DArray, Obstacle3D, Obstacle3DArray
from geometry_msgs.msg import Point
import message_filters
import ros2_numpy as rnp

import numpy as np
from scipy.spatial import cKDTree

IMAGE_SIZE_X = 1280
IMAGE_SIZE_Y = 720
FOCAL_X = 531.5750
FOCAL_Y = 531.3700
CENTER_X = 643.3700
CENTER_Y = 356.9830

DEPTH_MAX = 20.0
RANGE_MAX = 60.0
SEARCH_RADIUS = 0.75

INTRINSIC_MATRIX = np.array([
   [FOCAL_X, 0, CENTER_X, 0],
    [0, FOCAL_Y, CENTER_Y, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])

EXTRINSIC_MATRIX = np.array([
    [0, -1, 0, 0],
    [0, 0, -1,1],
    [1, 0, 0, -3.1],
    [0, 0, 0, 1]
])

PROJECTION_MATRIX = np.matmul(INTRINSIC_MATRIX, EXTRINSIC_MATRIX)
INV_PROJ = np.linalg.inv(PROJECTION_MATRIX)

CLASS_X = {
    0:  0.50,   # person
    1:  1.5,   # bicycle
    2:  4.5,    # car
    3:  2.0,   # motorbike
    4:  10.0,   # bus
    5:  5.0,    # truck
}


class BBoxGeneratorNode(Node):

    def __init__(self):
        super().__init__('bbox_gen_node')

        depth_sub = message_filters.Subscriber(self, Image, '/depth_image')
        obstacles_2d_sub = message_filters.Subscriber(self, Obstacle2DArray, '/obstacle_array_2d')
        self.sync = message_filters.ApproximateTimeSynchronizer([depth_sub, obstacles_2d_sub], 1, 2.0)
        self.sync.registerCallback(self.bbox_gen_cb)

        self.lidar_sub = self.create_subscription(
            PointCloud2,
            '/lidar_fused',
            self.lidar_cb,
            10
        )

        self.pcd_kdtree = None

        self.obstacles_3d_pub = self.create_publisher(
            Obstacle3DArray,
            '/obstacle_array_3d',
            10
        )
    

    def bbox_gen_cb(self, depth_msg: Image, obstacles_2d: Obstacle2DArray):
        self.get_logger().debug('got bbox_gen_cb')
        depth_image = rnp.numpify(depth_msg)
        obstacles_3d_array = Obstacle3DArray()

        for index, obstacle in enumerate(obstacles_2d.obstacles):
            # get the bounding box in pixels
            x1 = int(obstacle.bounding_box.x1 * IMAGE_SIZE_X)
            y1 = int(obstacle.bounding_box.y1 * IMAGE_SIZE_Y)
            x2 = int(obstacle.bounding_box.x2 * IMAGE_SIZE_X)
            y2 = int(obstacle.bounding_box.y2 * IMAGE_SIZE_Y)

            # get the approx min depth of bounding box
            min_depth = np.percentile(depth_image[y1:y2, x1:x2], 10)
            bbox_3d = np.zeros((8,3))

            # if the obstacle is further than max depth of camera
            if (min_depth > DEPTH_MAX):
                if self.pcd_kdtree != None: # pass the box generation to lidar-based method
                    # center of 2D bbox into 3D
                    center_pixel_x = (x2+x1) // 2
                    center_pixel_y = (y2+y1) // 2

                    largest_query = []
                    for z in np.arange(DEPTH_MAX, RANGE_MAX):
                        # project the center pixel of 2D bbox for different depth values
                        center_pixel = np.array([center_pixel_x, center_pixel_y, 1, 1/z]).reshape(4, -1)
                        proj_point = np.matmul(INV_PROJ, center_pixel) * z
                        proj_point = np.transpose(proj_point[:-1, :], (1, 0))

                        # get the nearest neighbor points to the projected center pixel within a radius
                        query_indices = self.pcd_kdtree.query_ball_point(proj_point, SEARCH_RADIUS)[0]

                        # store the query that resulted in most points
                        if len(query_indices) > len(largest_query):
                            largest_query = query_indices

                    # get the min depth of the largest cluster, which should be the object
                    largest_cluster = self.pcd_kdtree.data[largest_query]
                    if largest_cluster.size == 0:
                        continue
                    #min_depth = np.percentile(largest_cluster[:, 0], 10)
                    min_depth = largest_cluster[:, 0].min()
                    print(proj_point, largest_cluster)

                else:   # lidar data doesn't exist
                    continue

            # project 2-d bounding box into 3-d
            xx, yy = np.meshgrid([x1,x2], [y1,y2])
            depth_array = np.full_like(xx, min_depth, dtype=np.float32)
            pixel_array = np.stack([xx, yy, np.ones_like(xx), 1/(depth_array+1e-3)]).reshape(4,-1)
            bbox_3d_front = np.matmul(INV_PROJ, pixel_array) * depth_array.flatten()
            bbox_3d_front = np.transpose(bbox_3d_front[:-1, :], (1, 0)).astype(np.float32)

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

        obstacles_3d_array.header.frame_id = 'base_link'
        obstacles_3d_array.header.stamp = self.get_clock().now().to_msg()
        self.obstacles_3d_pub.publish(obstacles_3d_array)
        

    # expects transformed, fused lidar point clouds
    # filters the point cloud and stores it in a K-d Tree
    def lidar_cb(self, lidar_msg: PointCloud2):
        self.get_logger().debug('got lidar_cb')
        pcd = rnp.numpify(lidar_msg)
        if lidar_msg.header.frame_id != 'base_link':
            # something is wrong with frame_id
            self.get_logger().info('Received lidar scan with invalid frame')
            return
        
        # remove the floor 
        pcd = pcd[pcd['z'] > 0.2] 

        # get it into kdtree data format
        x, y, z = pcd['x'], pcd['y'], pcd['z']
        data = np.stack([x,y,z], axis=-1).reshape(-1, 3)

        # store as kdtree
        self.pcd_kdtree = cKDTree(data)

         
def main(args=None):
    rclpy.init(args=args)

    bbox_gen_node = BBoxGeneratorNode()

    rclpy.spin(bbox_gen_node)

    bbox_gen_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()