'''
Package:   carla_controller
Filename:  controller.py
Author:    Will Heitman (w at heit.mn)

Very simple controller for the CARLA leaderboard.
'''

from carla_msgs.msg import CarlaEgoVehicleControl
from rosgraph_msgs.msg import Clock
import rclpy
from rclpy.node import Node


class CarlaController(Node):
    def __init__(self):
        super().__init__('carla_controller')

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

        # self.control_timer = self.create_timer(0.05, self.generate_commands)

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

    carla_controller_node = CarlaController()

    rclpy.spin(carla_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    carla_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
