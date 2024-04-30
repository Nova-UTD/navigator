'''
Package: costs
   File: route_costmap_node.py
 Author: Justin Ruths

Subscribes to the global route and
publishes the local route as a cost map.
'''

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

# Message definitions
from navigator_msgs.msg import CarlaSpeedometer
from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Path
from navigator_msgs.msg import Egma
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, Point
from tf2_ros import LookupException, ExtrapolationException, TransformException, ConnectivityException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ros2_numpy.geometry import quat_to_numpy, numpy_to_quat
from tf_transformations import quaternion_multiply, euler_from_quaternion
from ros2_numpy.occupancy_grid import occupancygrid_to_numpy, numpy_to_occupancy_grid
# to do image rotations/shifts for fast forwarding occupancy grids
from scipy import ndimage

from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

from skimage.morphology import erosion

import matplotlib.pyplot as plt

class RouteCostmapNode(Node):

    def __init__(self):
        super().__init__('route_costmap_node')

        # Subscriptions and publishers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.route_dist_grid_pub = self.create_publisher(
            OccupancyGrid, '/grid/route_distance', 1)

        self.path_goal_pub = self.create_publisher(
            PoseStamped, '/planning/path_goal', 1)

        self.goal_marker_pub = self.create_publisher(
            Marker, '/planning/goal_marker', 1)

        route_sub = self.create_subscription(
            Path, '/planning/route', self.routeCb, 1)
        self.route = None
        self.not_visited = None

        # TODO: implement status book keeping
        # self.status_pub = self.create_publisher(
        #     DiagnosticStatus, '/node_status', 1)
        # self.status = DiagnosticStatus()

        self.costmap_timer = self.create_timer(0.05, self.buildRouteCostmap, callback_group=MutuallyExclusiveCallbackGroup())

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    # TODO: currently implemented, the route cannot be changed once it is first received
    def routeCb(self, msg: Path):
        if self.route is None:
            self.get_logger().debug('Received the route.')
            self.route = msg.poses
            self.route_remaining = msg.poses

    # TODO: this logic could be revisited.
    def buildRouteCostmap(self):
        # self.get_logger().info('Creating route costmap...')
        # assign some baseline cost for not following the route
        # TODO: Avoid hardcoding the costmap size
        routemap = np.zeros((151, 151)) + 25.0
        
        if self.route is None:
            self.get_logger().warning('Route Costmap Node has not received route yet.')
            self.publish(routemap,(0.0,0.0))
            return

        try:
            # get the transform from map to base_link
            ego_tf = self.tf_buffer.lookup_transform(
                "base_link",
                "map",
                rclpy.time.Time(),
                rclpy.time.Duration(seconds=5.0))
            
            roll, pitch, yaw = euler_from_quaternion(quat_to_numpy(ego_tf.transform.rotation))

            xmax = 40 # 40m in front of the car
            xmin = -20 # 20m in back of the car
            ymin = -30 # 30m left of the car
            ymax = 30 # 30m right of the car
            gridres = 0.4

            # transform the route points to base_link 
            route_baselink_x = np.zeros(len(self.route_remaining))
            route_baselink_y = np.zeros(len(self.route_remaining))
            # dist_to_car = np.zeros(len(self.route_remaining))
            for i,pose in enumerate(self.route_remaining):
                x = pose.pose.position.x*np.cos(yaw) - pose.pose.position.y*np.sin(yaw) + ego_tf.transform.translation.x
                y = pose.pose.position.x*np.sin(yaw) + pose.pose.position.y*np.cos(yaw) + ego_tf.transform.translation.y
                # dist_to_car[i] = np.sqrt(x**2+y**2)

                route_baselink_x[i] = x
                route_baselink_y[i] = y

            # Anything behind the vehicle we say we have visited already
            keep_idxs = np.flatnonzero(np.array(route_baselink_x) > 0) 

            # none of the remaining route is ahead of the vehicle, so we are done
            if len(keep_idxs) == 0:
                self.get_logger().warning('Did not find any route points ahead of the vehicle.')
                self.route_remaining = []
                self.publish(routemap,(0.0,0.0))
                return
            
            # the route may make some turns such that part of the future path goes behind the vehicle
            # so we keep everything starting with the first route point in front of the vehicle
            # select the one prior to this, so we can interpolate from that one to the one we care about
            start_idx = max(0,keep_idxs[0]-1) # pick the one prior to the first one

            # keep only the route starting from that index (adding in the origin, where the vehicle is)
            self.route_remaining = self.route_remaining[start_idx:]
            route_baselink_x = route_baselink_x[start_idx:]
            route_baselink_y = route_baselink_y[start_idx:]
            self.get_logger().debug('route has %i points' % len(route_baselink_x))

            # this loop interpolates between the route points so we have a point for every cell of the cost map
            # gridxs and gridys are in x/y coordinates - in meters
            gridxs = []
            gridys = []
            goal = None
            for i in range(1,len(route_baselink_x)):
                dx = route_baselink_x[i] - route_baselink_x[i-1]
                dy = route_baselink_y[i] - route_baselink_y[i-1]
                
                steps = int(np.ceil(max(abs(dx),abs(dy)) / gridres)) # find the axis that changes the most
                
                ts = np.linspace(0,1,steps+1)
                for t in ts[1:]:
                    newx = route_baselink_x[i-1] + t*dx
                    newy = route_baselink_y[i-1] + t*dy
                    # gridxs.append( newx )
                    # gridys.append( newy )
                    # the goal should be the last point in the route that is within the costmap
                    if self.is_within_costmap(newx,newy):
                        goal = (newx,newy)
                        gridxs.append( newx )
                        gridys.append( newy )
                    # else:
                    #     break

                # stop if we have left the costmap region
                # if not self.is_within_costmap(route_baselink_x[i],route_baselink_y[i]):
                #     break
            
            # self.get_logger().info('\n'+'\n'.join([ '%1.2f, %1.2f' % (gridxs[i],gridys[i]) for i in range(len(gridxs))] ) )
            self.get_logger().debug('grid route has %i points' % len(gridxs))
            self.get_logger().debug('path goal point:  %1.2f, %1.2f' % goal )

            # If we have only 1 or 0 in the list, there isn't really anything to show
            if len(gridxs) < 2:
                self.get_logger().info('You have reached the end of the route.')
                self.publish(routemap,(0.0,0.0))
                return
            
            # now we paint a low cost valley along the gridxs,gridys
            pixel_steps = [1,2,3,4] # how many costmap cells are painted to either "side" according to the cost_gradient below
            cost_gradient = [5,20,40,50]
            for r in range(len(gridxs)):
                # conversion from x,y in base_link to grid indices 
                # TODO: Avoid hard coding this
                i,j = round((gridys[r]+30)/0.4), round((gridxs[r]+20)/0.4)
                try:
                    routemap[i,j] = 0
                except:
                    continue
                if r > 1:
                    if jold<j: # last point is behind current one
                        for d in pixel_steps:
                            try: # all the try/excepts are to catch when i+d, i-d, j+d, j-d go outside the bounds of the costmap
                                routemap[i-d,j] = min(routemap[i-d,j],cost_gradient[d-1])
                                routemap[i+d,j] = min(routemap[i+d,j],cost_gradient[d-1])
                            except:
                                continue
                        if iold==i: # route is heading straight forward
                            pass
                        elif iold<i: # route is turning left                            
                            for d in pixel_steps:
                                try:
                                    routemap[i-d,j+d] = min(routemap[i-d,j+d],cost_gradient[d-1])
                                    routemap[i+d,j-d] = min(routemap[i+d,j-d],cost_gradient[d-1])
                                    routemap[i-d,j+d-1] = min(routemap[i-d,j+d-1],cost_gradient[d-1])
                                    routemap[i+d-1,j-d] = min(routemap[i+d-1,j-d],cost_gradient[d-1])
                                    routemap[i-d+1,j+d] = min(routemap[i-d+1,j+d],cost_gradient[d-1])
                                    routemap[i+d,j-d+1] = min(routemap[i+d,j-d+1],cost_gradient[d-1])
                                except:
                                    continue

                        elif iold>i: # route is turning right
                            for d in pixel_steps:
                                try:
                                    routemap[i-d,j-d] = min(routemap[i-d,j-d],cost_gradient[d-1])
                                    routemap[i+d,j+d] = min(routemap[i+d,j+d],cost_gradient[d-1])
                                    routemap[i-d+1,j-d] = min(routemap[i-d+1,j-d],cost_gradient[d-1])
                                    routemap[i+d-1,j+d] = min(routemap[i+d-1,j+d],cost_gradient[d-1])
                                    routemap[i-d,j-d+1] = min(routemap[i-d,j-d+1],cost_gradient[d-1])
                                    routemap[i+d,j+d-1] = min(routemap[i+d,j+d-1],cost_gradient[d-1])
                                except:
                                    continue
                                
                    else: # route is heading sideways
                        for d in pixel_steps:
                            try:
                                routemap[i,j-d] = min(routemap[i,j-d],cost_gradient[d-1])
                                routemap[i,j+d] = min(routemap[i,j+d],cost_gradient[d-1])
                            except:
                                continue
                    
                iold,jold = i,j

            self.publish(routemap, goal )

        except(LookupException, ExtrapolationException, ConnectivityException) as e: # typically get some errors on startup as the tf buffer fills
            self.get_logger().warning("!!! Error finding transform to build route grid !!!")
            self.get_logger().error('failed to get transform {} \n'.format(repr(e)))

    def publish(self, routemap, goal):
        # Publish path goal, which is the last element of the gridxs,gridys
        path_goal = PoseStamped()
        path_goal.header.stamp = self.clock.clock
        path_goal.header.frame_id = 'base_link'
        path_goal.pose.position.x = goal[0]
        path_goal.pose.position.y = goal[1]
        self.path_goal_pub.publish(path_goal)

        # create a marker for rviz
        self.publish_marker(path_goal,(0.0,1.0,0.4),self.goal_marker_pub)

        # create and combine radial gradient overlay
        #waypoint_costmap = self.make_waypoint_costmap(path_goal.pose)
        #routemap = np.clip( routemap + waypoint_costmap , 0, 100)

        # Publish as an OccupancyGrid
        route_cost_msg = OccupancyGrid()
        route_cost_msg.info.map_load_time = self.clock.clock
        route_cost_msg.info.resolution = 0.4 # TODO: should avoid hard coding this
        route_cost_msg.info.width = routemap.shape[0]
        route_cost_msg.info.height = routemap.shape[1]
        route_cost_msg.info.origin.position.x = -20.0 # TODO: should avoid hard coding this
        route_cost_msg.info.origin.position.y = -30.0 # TODO: should avoid hard coding this
        route_cost_msg.header.stamp = self.clock.clock
        route_cost_msg.header.frame_id = 'base_link'
        route_cost_msg.data = routemap.astype(np.int8).flatten().tolist()
        self.route_dist_grid_pub.publish(route_cost_msg)

    def is_within_costmap(self,x,y, xmin=-20.0,xmax=40.0,ymin=-30.0,ymax=30.0):
        # TODO: avoid hardcoding these values
        return x>=xmin and x<=xmax and y>=ymin and y<=ymax

    # this creates a radial costmap centered on the goal waypoint
    # creates a gradual cost landscape to drive the path towards the end
    def make_waypoint_costmap(self,waypoint):
        # TODO: avoid hardcoding these values
        xmax = 40 # 40m in front of the car
        xmin = -20 # 20m in back of the car
        ymin = -30 # 30m left of the car
        ymax = 30 # 30m right of the car
        gridres = 0.4
        
        costmap = np.zeros((151,151))
        for i in range(costmap.shape[0]):
            for j in range(costmap.shape[1]):
                x,y = gridres*j +xmin , gridres*i + ymin
                dist = np.sqrt( (x-waypoint.position.x)**2 + (y-waypoint.position.y)**2 )
                costmap[i,j] = 50.0 * dist / np.sqrt(2*60.0**2)

        return costmap

    def publish_marker(self, target, c, publisher):
        marker = Marker()
        marker.header.frame_id = 'base_link'
        marker.header.stamp = self.clock.clock
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.scale.x = 0.5
        marker.scale.y = 1.0
        marker.scale.z = 0.75

        color = ColorRGBA()
        color.a = 0.85
        color.r = c[0]
        color.g = c[1]
        color.b = c[2]
        marker.color = color

        pt_a = Point()
        marker.points.append(pt_a)

        pt_b = Point()
        pt_b.x = target.pose.position.x
        pt_b.y = target.pose.position.y
        pt_b.z = 0.3
        marker.points.append(pt_b)

        publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RouteCostmapNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
