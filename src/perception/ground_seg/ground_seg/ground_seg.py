import numpy as np
import ros2_numpy as rnp
import math
import itertools
import copy

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String


class GroundSeg(Node):

    def __init__(self):
        super().__init__('ground_seg')

        #===CONSTANTS===#
        self.LiDAR_HEIGHT = -1.73  # initialize to the z-height of the LiDAR (in meters)
        self.CAR_HEIGHT = 2  # initialize to the z-height of car (in meters)
        self.RES = 1./3.
        self.S = 0.09

        #===PARAMETERS===#
        self.lidar_sub = self.create_subscription(PointCloud2, '/lidar_front/points_raw', self.simpl_ground_seg, 10)
        self.ground_seg_pts_pub = self.create_publisher(PointCloud2, 'ground_seg_points', 10)

    #===================IGNOR=============================================
        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        #timer_period = 0.5  # seconds
        #self.timer = self.create_timer(timer_period, self.timer_callback)
        #self.i = 0
    
    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1
    #=====================================================================

    def simpl_ground_seg(self, msg: PointCloud2):
        pcd = rnp.numpify(msg)
        pcd = pcd[pcd['z'] >= -self.CAR_HEIGHT]

        #return point_cloud_seg
        self.get_logger().info('Publishing: simpl ground seg is DONE')
        self.ground_seg_pts_pub.publish(pcd)

    def ground_seg(self, msg: PointCloud2, res: float =None, s: float =None):

        # make an array of points from PointCloud2. 
        # ex: PointClouyd2 array --> [[x1,y1,z1], [x2,y2,z2],...]
        point_cloud = rnp.numpify(msg)

        # initializing parameters if none are given
        if res == None:
            res = self.RES

        if s == None:
            s = self.S

        num_points = point_cloud.shape[0]

        # generate 2-D grid of the LiDAR cloud
        max_index = math.sqrt(2.)*(128/3./2.+1.)

        # a 2D array that contains lists of 3D points in point_cloud that map to
        # a particular grid cell (according to the place of the 3D point in point_cloud)
        # a math.ceil is rounding numbers up, for example math.ceil(4.3) = 5 and math.ceil(-3.3) = -3
        filler = np.frompyfunc(lambda x: list(), 1, 1)
        grid = np.empty((int(2 * math.ceil(max_index/res) + 1), int(2 * math.ceil(max_index/res) + 1)), dtype=np.object)
        filler(grid, grid);

        # determine the center coordinate of the 2D grid
        center_x = int(math.ceil(max_index/res))
        center_y = int(math.ceil(max_index/res))

        for i in range(num_points):
            point = point_cloud[i,:]
            #self.get_logger().info('Publishing: "%s"' % msg)
            #self.get_logger().info('Publishing: "%s"' % point_cloud)
            self.get_logger().info('Publishing: "%s"' % num_points)
            self.get_logger().info('Publishing: "%s"' % point)
            x = point[0]
            y = point[1]
            z = point[2]

            if ((math.fabs(x) <= max_index) and (math.fabs(y) <= max_index) and (z <= 3.5)):

                grid[int(center_x + round(x/res)), int(center_y + round(y/res))].append(i)

        h_G = np.nan*np.empty((grid.shape))

        # iterate radially outwards to compute if a point belongs to the ground (1) on mask grid
        grid_seg = np.zeros(grid.shape)

        # initialize the center coordinate of the 2D grid to ground
        points_z = np.ndarray.tolist(point_cloud[grid[center_x, center_y],2])
        H = max(points_z or [np.nan])

        if not math.isnan(H):
            h_G[center_x, center_y] = H
        else:
            h_G[center_x, center_y] = self.LiDAR_HEIGHT

        # initialize the coordinates of inner circle
        circle_inner = [[center_x, center_y]]

        # identify all the points that were labeled as not ground
        point_cloud_seg = np.empty((0,3))

        for i in range(1,int(math.ceil(max_index/res))+1):

            # generate indices at the ith inner circle level
            circle_curr = self.generate_circle(i, center_x, center_y)

            for indices in circle_curr:
                x = indices[0]
                y = indices[1]

                # compute h_hat_G: find max h_G of neighbors
                neigh_indeces = np.array(self.get_neighbors(x,y,circle_inner))

                # compute the min and max z coordinates of each grid cell
                points_z = np.ndarray.tolist(point_cloud[grid[x,y],2])
                H = max(points_z or [np.nan])
                h = min(points_z or [np.nan])

                h_hat_G = np.nanmax(h_G[neigh_indeces])

                if ((not np.isnan(H)) and (not np.isnan(h)) and \
                    (H - h < s) and (H - h_hat_G < s)):
                    grid_seg[x,y] = 1
                    h_G[x,y] = copy.deepcopy(H)

                else:

                    h_G[x,y] = copy.deepcopy(h_hat_G)

                    # add to not ground points
                    point_locations = grid[x,y]

                    if point_locations != []:
                        point_cloud_seg = np.vstack((point_cloud_seg,point_cloud[point_locations,:]))

            # update the inner circle indices
            circle_inner = copy.deepcopy(circle_curr)

        #return point_cloud_seg
        self.ground_seg_pts_pub.publish(self.array_to_msg(point_cloud_seg))

    # return the indices of a circle at level i from the center of the grid
    def generate_circle(self, i, center_x, center_y):

        circle_range = range(-1*i,i+1)
        circle = [list(x) for x in itertools.product(circle_range, circle_range)]
        circle = [[item[0]+center_x, item[1]+center_y] for item in circle if ((abs(item[0]) == i) or (abs(item[1]) == i))]

        return circle

    # get the inner circle neighbors of a point
    def get_neighbors(self, x, y, circle_inner):
        neigh_indices = []
        for indices in circle_inner:
            if ((abs(x-indices[0]) < 2) and (abs(y-indices[1]) < 2)):
                neigh_indices.append(indices)

        return neigh_indices

    def array_to_msg(nparray: np.array):
        data = np.zeros(len(nparray), dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32)
        ])
        data['x'] = nparray[:,0]
        data['y'] = nparray[:,1]
        data['z'] = nparray[:,2]

        msg = ros2_numpy.msgify(PointCloud2, data, stamp=header.stamp, frame_id=header.frame_id)

        return msg

def main(args=None):
    rclpy.init(args=args)

    ground_seg = GroundSeg()

    rclpy.spin(ground_seg)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ground_seg.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

