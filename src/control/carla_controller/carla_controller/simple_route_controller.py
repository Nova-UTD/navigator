'''
Package:   carla_controller
Filename:  controller.py
Author:    Will Heitman (w at heit.mn)

Very simple controller for the CARLA leaderboard.
'''

from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import shapely


class SimpleRouteControllerNode(Node):
    def __init__(self):
        super().__init__('simple_route_controller')

        # Command publisher
        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl,
            '/carla/hero/vehicle_control_cmd',  # Must be on this topic
            10
        )

        # Clock subscription
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self._tick_, 10)
        self._cached_clock_ = Clock()

        # Path subscription
        self.path_sub = self.create_subscription(
            Path, '/route/smooth_path', self.path_cb, 10)
        self.path_msg = Path()

        # Transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_linestring = None

        # self.control_timer = self.create_timer(0.05, self.generate_commands)

    def path_cb(self, msg: Path):
        self.get_logger().info("Received smooth path")
        self.path_msg = msg

        if self.path_linestring is None:
            # Construct a shapely LineString
            pts = []
            for pose in msg.poses:
                pts.append(np.array([pose.pose.position.x, pose.pose.position.y]))
                
            self.route_pts_arr = np.array(pts)

            print(str(self.route_pts_arr))

        # Get map->base_link transform
        t = TransformStamped()
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                'map',
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform from map to base_link: {ex}')
            return
        
        # Transform linestring to base_link
        self.route_pts_arr[:,0] += t.transform.translation.x
        self.route_pts_arr[:,1] += t.transform.translation.y

        linestring = shapely.LineString(self.route_pts_arr)
        current_pos = shapely.Point(0.0, 0.0) # Our vehicle is at the origin of base_link, by definition

        distance = shapely.distance(linestring, current_pos)
        self.get_logger().info(str(distance))
        

    def generate_commands(self):

        # Form a blank command message
        # https://github.com/carla-simulator/ros-carla-msgs/blob/leaderboard-2.0/msg/CarlaEgoVehicleControl.msg
        command = CarlaEgoVehicleControl()

        # Form our header, including current time
        command.header.frame_id = 'base_link'
        command.header.stamp = self._cached_clock_.clock

        # If the current time in seconds is even, drive forward
        # Else drive backward
        if self._cached_clock_.clock.sec % 2 == 0:
            command.reverse = False
        else:
            command.reverse = True

        # Set the throttle to zero (don't move)
        command.throttle = 0.0

        # Pubish our finished command
        self.command_pub.publish(command)

    def _tick_(self, msg: Clock):
        self._cached_clock_ = msg
        self.generate_commands()


def main(args=None):
    rclpy.init(args=args)

    carla_controller_node = SimpleRouteControllerNode()

    rclpy.spin(carla_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    carla_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
