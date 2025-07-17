'''
Package: grids
   File: grid_summation_node.py
 Author: Will Heitman (w at heit dot mn)

Subscribes to cost maps, calculates their weighted sum, and
publishes the result as a finished cost map.
'''

import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time
import yaml

from diagnostic_msgs.msg import DiagnosticStatus
from nav_msgs.msg import OccupancyGrid
from navigator_msgs.msg import Egma
from rosgraph_msgs.msg import Clock
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ros2_numpy.geometry import quat_to_numpy, numpy_to_quat
from tf_transformations import quaternion_multiply, euler_from_quaternion
from ros2_numpy.occupancy_grid import occupancygrid_to_numpy

from scipy import ndimage

import matplotlib.pyplot as plt


STALENESS_TOLERANCE = 0.25  # seconds. Grids older than this will be ignored.

CURRENT_OCCUPANCY_SCALE = 1.0 # 0.75
FUTURE_OCCUPANCY_SCALE = 1.0 #3.0
DRIVABLE_GRID_SCALE = 1.0 #0.75
ROUTE_DISTANCE_GRID_SCALE = 1.0
JUNCTION_GRID_SCALE = 1.0


class GridSummationNode(Node):

    def __init__(self):
        """Subscribe to the desired cost maps

        - Drivable surface  (~5 Hz)
        - Route distance
        - Current occupancy (~8 Hz)

        """
        super().__init__('grid_summation_node')

        # Set up the global config file
        self.declare_parameter('global_config', 'temp_value')
        self.file_path = self.get_parameter('global_config').value

        # Subscriptions and publishers
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.current_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/occupancy/current', self.currentOccupancyCb, 1)
        self.current_occupancy_grid = None

        self.future_occupancy_sub = self.create_subscription(
            OccupancyGrid, '/grid/predictions_combined', self.futureOccupancyCb, 1)
        self.future_occupancy_grid = None

        self.drivable_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/drivable', self.drivableGridCb, 1)
        self.drivable_grid = None

        self.junction_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/stateful_junction', self.junctionGridCb, 1)
        self.junction_grid = None

        self.route_dist_grid_sub = self.create_subscription(
            OccupancyGrid, '/grid/route_distance', self.routeDistGridCb, 1)
        self.route_dist_grid = None

        self.steering_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/steering_cost', 1)

        self.speed_cost_pub = self.create_publisher(
            OccupancyGrid, '/grid/speed_cost', 1)

        self.combined_egma_pub = self.create_publisher(
            Egma, '/egma/cost', 1)

        self.status_pub = self.create_publisher(
            DiagnosticStatus, '/node_status', 1)
        self.status = DiagnosticStatus()
        self.status.level = DiagnosticStatus.OK

        self.combine_timer = self.create_timer(0.05, self.createCostMap, callback_group=MutuallyExclusiveCallbackGroup())

        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clockCb, 1)

        self.clock = Clock()

    def clockCb(self, msg: Clock):
        self.clock = msg

    # Make sure we're only keeping the newest grid messages
    def currentOccupancyCb(self, msg: OccupancyGrid):
        if self.current_occupancy_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.current_occupancy_grid.header.stamp.sec + self.current_occupancy_grid.header.stamp.nanosec*1e-9:
            self.current_occupancy_grid = msg

    def futureOccupancyCb(self, msg: OccupancyGrid):
        if self.future_occupancy_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.future_occupancy_grid.header.stamp.sec + self.future_occupancy_grid.header.stamp.nanosec*1e-9:
            self.future_occupancy_grid = msg

    def drivableGridCb(self, msg: OccupancyGrid):
        if self.drivable_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.drivable_grid.header.stamp.sec + self.drivable_grid.header.stamp.nanosec*1e-9:
            self.drivable_grid = msg

    def junctionGridCb(self, msg: OccupancyGrid):
        if self.junction_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.junction_grid.header.stamp.sec + self.junction_grid.header.stamp.nanosec*1e-9:
            self.junction_grid = msg

    def routeDistGridCb(self, msg: OccupancyGrid):
        if self.route_dist_grid is None or msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 > self.route_dist_grid.header.stamp.sec + self.route_dist_grid.header.stamp.nanosec*1e-9:
            self.route_dist_grid = msg

    def checkForStaleness(self, grid: OccupancyGrid):
        stamp = grid.header.stamp
        stamp_in_seconds = stamp.sec + stamp.nanosec*1e-9
        current_time_in_seconds = self.clock.clock.sec + self.clock.clock.nanosec*1e-9

        stale_time = current_time_in_seconds - stamp_in_seconds
        stale = stale_time > STALENESS_TOLERANCE
        print("Stale for " + str(stale_time) + " seconds")
        
        if stale:
            self.status.level = DiagnosticStatus.WARN
            self.status.message = "Current occupancy was stale."
            return current_time_in_seconds - stamp_in_seconds
        else:
            return 0

    def fastforward(self, grid: OccupancyGrid):
        if grid is None:
            return grid
        
        # convert the grid to numpy array, we'll treat it as an image
        grid_img = occupancygrid_to_numpy(grid)
        old_dim = grid_img.shape[0]
        grid_res = grid.info.resolution

        if old_dim == 128:  
            dim_bef_resize = old_dim

            grid_img = self.resizeOccupancyGrid(grid_img)
            old_dim = grid_img.shape[0]
            grid_res = dim_bef_resize * grid_res / old_dim

        # find that transform between base_link frames from when the message was made until now
        t = self.tf_buffer.lookup_transform_full(
            target_frame='base_link',
            target_time=rclpy.time.Time(),  # this requests the most recent time available, self.clock.clock might ask for a time more recent that we have data for...
            source_frame='base_link',
            source_time=grid.header.stamp,
            fixed_frame='map',
            timeout=rclpy.duration.Duration(seconds=5.0))

        roll, pitch, yaw = euler_from_quaternion(quat_to_numpy(t.transform.rotation))

        # Open the config file
        try:
            with open(self.file_path, 'r') as file:
                data = yaml.safe_load(file)
        except FileNotFoundError:
            print("Error: config.yaml not found.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML file: {e}")

        # use 10 here because it is the distance from the center of the occupancy grid to the vehicle
        x = t.transform.translation.x + ((0.5 * data['occupancy_grids']['vehicle_y_location']) - (0.5 * data['occupancy_grids']['vehicle_y_location'])*np.cos(-yaw))
        y = t.transform.translation.y - (0.5 * data['occupancy_grids']['vehicle_y_location'])*np.sin(-yaw)
        
        shift = [y/grid_res, x/grid_res]

        grid_img = ndimage.rotate(grid_img, np.degrees(-yaw), reshape=True)
        new_dim = grid_img.shape[0]
        diff = int((new_dim-old_dim)/2)
        grid_img = ndimage.shift(grid_img,shift)
        grid_img = grid_img[diff:new_dim-diff,diff:new_dim-diff]

        # sometimes grid_img emerges with 152 pixels..
        prezoom_rows = grid_img.shape[0]
        if prezoom_rows != 151:
            grid_img = ndimage.zoom(grid_img, 151.0/float(grid_img.shape[0]))

        grid_out = OccupancyGrid()
        grid_out.info.map_load_time = self.clock.clock
        grid_out.info.resolution = (grid_res * old_dim / (new_dim - diff)) * prezoom_rows / grid_img.shape[0]
        grid_out.info.width = grid_img.shape[0]
        grid_out.info.height = grid_img.shape[1]
        grid_out.info.origin.position.x = grid_img.shape[0] * grid_out.info.resolution * 2 / 3 * -1
        grid_out.info.origin.position.y = grid_img.shape[1] * grid_out.info.resolution * 1 / 2 * -1
        grid_out.header.stamp = self.clock.clock
        grid_out.header.frame_id = 'base_link'
        grid_out.data = grid_img.astype(np.int8).flatten().tolist()

        return grid_out

    def getWeightedArrayFromNumpy(self, msg: OccupancyGrid, scale: float) -> np.ndarray:
        """Converts the OccupancyGrid message into a numpy array, then multiplies it by scale

        Args:
            msg (OccupancyGrid)
            scale (float)

        Returns:
            np.ndarray: Weighted ndarray
        """
        height, width = msg.shape

        arr = np.asarray(msg.data, dtype=np.float16).reshape(height, width)

        arr *= scale

        return arr

    def getWeightedArrayFromOccupancyGrid(self, msg: OccupancyGrid, scale: float) -> np.ndarray:
        """Converts the OccupancyGrid message into a numpy array, then multiplies it by scale

        Args:
            msg (OccupancyGrid)
            scale (float)

        Returns:
            np.ndarray: Weighted ndarray
        """
        arr = np.asarray(msg.data, dtype=np.float16).reshape(msg.info.height, msg.info.width)

        arr *= scale

        return arr

    def resizeOccupancyGrid(self, original: np.ndarray) -> np.ndarray:
        # Remove every 6th row
        rows_to_delete = np.arange(0, original.shape[0], 6)
        downsampled = np.delete(original, rows_to_delete, axis=0)

        # Remove every 6th column
        cols_to_delete = np.arange(0, original.shape[1], 6)
        downsampled = np.delete(downsampled, cols_to_delete, axis=1)

        # trim 3 columns from the left (behind the car)
        downsampled = downsampled[:, 3:]

        # Now make sure downsampled has the correct shape for the background
        background = np.zeros((151, 151))
        h, w = downsampled.shape
        background[22:22+h, 0:w] = downsampled
        
        return background

    def createCostMap(self):
        # self.get_logger().info('Composing aggregate costmap...')
        steering_cost = np.zeros((151, 151))
        speed_cost = np.zeros((151, 151))

        # Calculate the weighted cost map layers
        grids = [('occupancy', self.current_occupancy_grid, CURRENT_OCCUPANCY_SCALE),
                 ('future_occupancy', self.future_occupancy_grid, FUTURE_OCCUPANCY_SCALE),
                 ('drivable', self.drivable_grid, DRIVABLE_GRID_SCALE),
                 ('route_dist', self.route_dist_grid, ROUTE_DISTANCE_GRID_SCALE),
                 ('junction', self.junction_grid, JUNCTION_GRID_SCALE)
                ] 

        try:
            for grid_name, grid, scale in grids:
                if grid is None or len(grid.data) == 0:
                    print("GRID NOT FOUND")
                    continue
                
                stale = self.checkForStaleness(grid)
                if stale > 0:
                    ff_grid = self.fastforward(grid)
                    weighted_grid_arr = self.getWeightedArrayFromOccupancyGrid(ff_grid, scale)
                else:
                    ff_grid = occupancygrid_to_numpy(grid)
                    if grid_name == 'occupancy' or grid_name == 'future_occupancy':
                        ff_grid = self.resizeOccupancyGrid(ff_grid)
                    weighted_grid_arr = ff_grid*scale

                if isinstance(ff_grid, np.ndarray):
                    weighted_grid_arr = self.getWeightedArrayFromNumpy(ff_grid, scale)
                else:
                    weighted_grid_arr = self.getWeightedArrayFromOccupancyGrid(ff_grid, scale)
                
                if grid_name == 'occupancy' or grid_name == 'future_occupancy':
                    weighted_grid_arr = self.resizeOccupancyGrid(weighted_grid_arr)

                if grid_name == 'drivable':
                    steering_cost = np.maximum( steering_cost , weighted_grid_arr )
                elif grid_name == 'junction':
                    speed_cost = np.maximum( speed_cost , weighted_grid_arr )
                else:
                    steering_cost = np.maximum( steering_cost , weighted_grid_arr )

            # Cap this to 100
            steering_cost = np.clip(steering_cost, 0, 100)
            speed_cost = np.clip(speed_cost, 0, 100)

            # Open the config file
            try:
                with open(self.file_path, 'r') as file:
                    data = yaml.safe_load(file)
            except FileNotFoundError:
                print("Error: config.yaml not found.")
            except yaml.YAMLError as e:
                print(f"Error parsing YAML file: {e}")

            # Publish as an OccupancyGrid
            steering_cost_msg = OccupancyGrid()
            steering_cost_msg.info.map_load_time = self.clock.clock
            steering_cost_msg.info.resolution = data['occupancy_grids']['resolution']
            steering_cost_msg.info.width = steering_cost.shape[1]
            steering_cost_msg.info.height = steering_cost.shape[0]
            steering_cost_msg.info.origin.position.x = -1 * data['occupancy_grids']['vehicle_x_location']
            steering_cost_msg.info.origin.position.y = -1 * data['occupancy_grids']['vehicle_y_location']
            steering_cost_msg.header.stamp = self.clock.clock
            steering_cost_msg.header.frame_id = 'base_link'
            steering_cost_msg.data = steering_cost.astype(np.int8).flatten().tolist()

            # Resize occupancy grid to match size specified in config file
            if steering_cost_msg.info.height != data['occupancy_grids']['length']:
                diff = (data['occupancy_grids']['length'] - steering_cost_msg.info.height) / steering_cost_msg.info.resolution
                diff = int(diff)  # Convert to integer
                
                if diff < 0:
                    steering_cost_msg.data = steering_cost_msg.data[:diff * steering_cost_msg.info.width]
                elif diff > 0:
                    steering_cost_msg.data.extend(Array(np.int8, [-1] * (diff * steering_cost_msg.info.width)))
                
                steering_cost_msg.info.height = data['occupancy_grids']['length']
            
            if steering_cost_msg.info.width != data['occupancy_grids']['width']:
                diff = (data['occupancy_grids']['width'] - steering_cost_msg.info.width) / steering_cost_msg.info.resolution     
                diff = int(diff)  # Convert to integer
            
                if diff < 0:
                    steering_cost_msg.data = steering_cost_msg.data[:, int(diff / 2):(int(diff / 2) * -1)]
                elif diff > 0:
                    new_grid = Array(np.int8, [-1] * (data['occupancy_grids']['width'] * steering_cost_msg.info.height))
                    offset = int(diff / 2)
                    for i in range(steering_cost_msg.info.height):
                        start = i * data['occupancy_grids']['width'] + offset
                        end = ((i + 1) * data['occupancy_grids']['width']) - 1 - offset
                        new_grid[start:end] = steering_cost_msg.data[i * steering_cost_msg.info.width:(i + 1) * steering_cost_msg.info.width]
                    steering_cost_msg.data = new_grid
            
                steering_cost_msg.info.width = data['occupancy_grids']['width']

            self.steering_cost_pub.publish(steering_cost_msg)

            speed_cost_msg = OccupancyGrid()
            speed_cost_msg.info.map_load_time = self.clock.clock
            speed_cost_msg.info.resolution = data['occupancy_grids']['resolution']
            speed_cost_msg.info.width = speed_cost.shape[1]
            speed_cost_msg.info.height = speed_cost.shape[0]
            speed_cost_msg.info.origin.position.x = -1 * data['occupancy_grids']['vehicle_x_location']
            speed_cost_msg.info.origin.position.y = -1 * data['occupancy_grids']['vehicle_y_location']
            speed_cost_msg.header.stamp = self.clock.clock
            speed_cost_msg.header.frame_id = 'base_link'
            speed_cost_msg.data = speed_cost.astype(np.int8).flatten().tolist()

            # Resize occupancy grid to match size specified in config file
            if speed_cost_msg.info.height != data['occupancy_grids']['length']:
                diff = (data['occupancy_grids']['length'] - speed_cost_msg.info.height) / speed_cost_msg.info.resolution
                diff = int(diff)  # Convert to integer
                
                if diff < 0:
                    speed_cost_msg.data = speed_cost_msg.data[:diff * speed_cost_msg.info.width]
                elif diff > 0:
                    speed_cost_msg.data.extend(Array(np.int8, [-1] * (diff * speed_cost_msg.info.width)))
                
                speed_cost_msg.info.height = data['occupancy_grids']['length']
            
            if speed_cost_msg.info.width != data['occupancy_grids']['width']:
                diff = (data['occupancy_grids']['width'] - speed_cost_msg.info.width) / speed_cost_msg.info.resolution 
                diff = int(diff)  # Convert to integer
            
                if diff < 0:
                    speed_cost_msg.data = speed_cost_msg.data[:, int(diff / 2):(int(diff / 2) * -1)]
                elif diff > 0:
                    new_grid = Array(np.int8, [-1] * (data['occupancy_grids']['width'] * speed_cost_msg.info.height))
                    offset = int(diff / 2)
                    for i in range(speed_cost_msg.info.height):
                        start = i * data['occupancy_grids']['width'] + offset
                        end = ((i + 1) * data['occupancy_grids']['width']) - 1 - offset
                        new_grid[start:end] = speed_cost_msg.data[i * speed_cost_msg.info.width:(i + 1) * speed_cost_msg.info.width]
                    speed_cost_msg.data = new_grid
            
                speed_cost_msg.info.width = data['occupancy_grids']['width']

            self.speed_cost_pub.publish(speed_cost_msg)
        
        except(Exception) as e:
            self.get_logger().warn('Error composing aggregate cost map - likely waiting for Transform Buffer.')
            self.get_logger().warn(str(e))
        
"""
        #Publish Egma
        egma_msg = Egma()
        egma_msg.header.stamp = self.clock.clock
        egma_msg.header.frame_id = 'base_link'
        current_stamp = egma_msg.header.stamp
        t = current_stamp.sec + current_stamp.nanosec * 1e-9
        for i in range(15):
            frame = steering_cost_msg
            t += 0.1  
            next_stamp = current_stamp
            next_stamp.sec = int(t)
            next_stamp.nanosec = int(t * 1e9 % 1e9)
            steering_cost_msg.header.stamp = next_stamp
            steering_cost_msg.header.frame_id = 'base_link'
            egma_msg.egma.append(frame)
        self.combined_egma_pub.publish(egma_msg)
"""

def main(args=None):
    rclpy.init(args=args)
    node = GridSummationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

