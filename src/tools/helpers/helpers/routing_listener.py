'''
Package:   helpers
Filename:  routing_listener.py
Author:    Will Heitman (w at heit.mn)

Subscribes to:
/clicked_point (geometry_msgs/msg/PointStamped, published by Rviz2)
/tf

Calls:
/set_route (navigator_msgs/srv/SetRoute, served by MapManager)
'''

import math

from geometry_msgs.msg import Point, PointStamped
from navigator_msgs.srv import SetRoute
from rclpy.node import Node
import rclpy
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener



class routing_listener_node(Node):

    def __init__(self):
        super().__init__('routing_listener_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        clicked_point_sub = self.create_subscription(PointStamped, '/clicked_point', self.clickedPointCb, 1)

        self.routing_client = self.create_client(SetRoute, '/set_route')

    def clickedPointCb(self, msg: PointStamped):
        service_request = SetRoute.Request()

        start = Point()
        destination = Point()

        # Look up our current position, which will be the route start
        try:
            ego_tf = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time())

            start.x = ego_tf.transform.translation.x
            start.y = ego_tf.transform.translation.y
            start.z = ego_tf.transform.translation.z
        except TransformException as ex:
            self.get_logger().info(
                f'Could not find base_link->map transform.')
            return

        # Look up the clicked point's transform in the map frame
        try:
            clicked_point_tf = self.tf_buffer.lookup_transform(
                "map",
                msg.header.frame_id,
                rclpy.time.Time())
            if msg.header.frame_id == "map":
                destination = msg.point
            else:
                # Apply the transform.
                # First rotate
                yaw = math.acos(clicked_point_tf.transform.rotation.w) * 2
                x_ = msg.point.x
                y_ = msg.point.y
                destination.x = x_ * math.cos(yaw) - y_ * math.sin(yaw)
                destination.y = y_ * math.cos(yaw) + x_ * math.sin(yaw)
                
                # Then translate
                destination.x += clickked_point_tf.transform.translation.x
                destination.y += clicked_point_tf.transform.translation.y
        except TransformException as ex:
            self.get_logger().info(
                f'Could not find {msg.header.frame_id}->map transform.')
            return
        
        service_request.route_nodes.append(start)
        service_request.route_nodes.append(destination)

        self.get_logger().info(f"Requesting route from ({start.x}, {start.y}) to ({destination.x}, {destination.y})")
        # response: SetRoute.Response = self.routing_client.call_async(service_request)
        # self.get_logger().info(f"Got response {response}")

        future = self.routing_client.call_async(service_request)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Got response {future.result}")





def main(args=None):
    rclpy.init(args=args)
    routing_listener = routing_listener_node()
    rclpy.spin(routing_listener)
    routing_listener.destroy_node()
    rclpy.shutdown()