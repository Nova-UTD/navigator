'''
Package:   helpers
Filename:  routing_monitor.py

Subscribes to:
/planning/rough_route (Path, published by an interface - CARLA, Real World)
/tf

Calls:
/set_route (navigator_msgs/srv/SetRoute, served by MapManager)
'''

import math

from geometry_msgs.msg import Point, PointStamped, PoseStamped
from navigator_msgs.srv import SetRoute
from nav_msgs.msg import Path
from rclpy.node import Node
import rclpy
import rclpy.time
import rclpy.utilities
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rosgraph_msgs.msg import Clock
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

parallel_group = ReentrantCallbackGroup()
mutex_group = MutuallyExclusiveCallbackGroup()

class RoutingMonitor(Node):

    def __init__(self):
        super().__init__('routing_monitor_node')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        rough_route_sub = self.create_subscription(Path, '/planning/rough_route', self.routeCb, 1, callback_group=parallel_group)
        self.rough_route = None

        # TODO: should implement this as a service rather than two similar/redundant topics...
        # currently doing this so that the fast republishing does not need to lie with the map manager
        smooth_route_sub = self.create_subscription(Path, '/planning/smoothed_route', self.smooth_routeCb, 1, callback_group=parallel_group)
        self.smooth_route_msg = None
        self.smooth_route_pub = self.create_publisher(Path, '/planning/route', 1, callback_group=parallel_group)

        self.routing_client = self.create_client(SetRoute, '/set_route', callback_group=parallel_group)

        smooth_route_timer = self.create_timer(0.1, self.smooth_route_pub_tick, callback_group=parallel_group)

        clock_sub = self.create_subscription(Clock, '/clock', self.clockCb, 1, callback_group=parallel_group)
        self.clock = Clock().clock

        self.service_request = SetRoute.Request()
        self.route_timer = self.create_timer(3.0, self.request_refined_route, callback_group=mutex_group)

    def clockCb(self, msg: Clock):
        self.clock = msg.clock

    def routeCb(self, msg: Path):
        self.rough_route = msg

    def smooth_routeCb(self, msg: Path):
        if self.smooth_route_msg is None:
            self.get_logger().info('Smooth Route received.')
        self.smooth_route_msg = msg

    def smooth_route_pub_tick(self):
        if self.smooth_route_msg is not None:
            self.smooth_route_msg.header.stamp = self.clock
            for i in range(len(self.smooth_route_msg.poses)):
                self.smooth_route_msg.poses[i].header.stamp = self.clock
            self.smooth_route_pub.publish(self.smooth_route_msg)

    def request_refined_route(self):
        if self.rough_route is None:
            self.get_logger().info("Rough Route was not yet received")
            return

        start = Point()
        start.x = self.rough_route.poses[0].pose.position.x
        start.y = self.rough_route.poses[0].pose.position.y
        destination = Point()
        destination.x = self.rough_route.poses[-1].pose.position.x
        destination.y = self.rough_route.poses[-1].pose.position.y

        # # Look up our current position, which will be the route start
        # try:
        #     ego_tf = self.tf_buffer.lookup_transform(
        #         "map",
        #         "base_link",
        #         rclpy.time.Time())

        #     start.x = ego_tf.transform.translation.x
        #     start.y = ego_tf.transform.translation.y
        #     start.z = ego_tf.transform.translation.z
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not find base_link->map transform.')
        #     return

        # # Look up the clicked point's transform in the map frame
        # try:
        #     clicked_point_tf = self.tf_buffer.lookup_transform(
        #         "map",
        #         msg.header.frame_id,
        #         rclpy.time.Time())
        #     if msg.header.frame_id == "map":
        #         destination = msg.point
        #     else:
        #         # Apply the transform.
        #         # First rotate
        #         yaw = math.acos(clicked_point_tf.transform.rotation.w) * 2
        #         x_ = msg.point.x
        #         y_ = msg.point.y
        #         destination.x = x_ * math.cos(yaw) - y_ * math.sin(yaw)
        #         destination.y = y_ * math.cos(yaw) + x_ * math.sin(yaw)
                
        #         # Then translate
        #         destination.x += clickked_point_tf.transform.translation.x
        #         destination.y += clicked_point_tf.transform.translation.y
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not find {msg.header.frame_id}->map transform.')
        #     return
        
        self.service_request.route_nodes.append(start)
        self.service_request.route_nodes.append(destination)

        self.get_logger().info(f"Requesting route from ({start.x}, {start.y}) to ({destination.x}, {destination.y})")
        # response: SetRoute.Response = self.routing_client.call_async(service_request)
        # self.get_logger().info(f"Got response {response}")

        self.future = self.routing_client.call_async(self.service_request)
        rclpy.spin_until_future_complete(self, self.future, self.executor, 3.0)
        result = self.future.result()
        self.get_logger().info("Got response %s %s" % (str(result.message),str(result.success)))
        if result.success:
            self.get_logger().info("Route was set successfully, moving on.")
            # to do this only once...
            self.route_timer.destroy()
        else:
            self.get_logger().info("Route failed to be set successfully, will retry on next timer trigger...")
        
        # Re-add self to executor after call_async removes it
        self.executor.add_node(self)



def main(args=None):
    rclpy.init(args=args)
    routing_monitor = RoutingMonitor()
    routing_executor = MultiThreadedExecutor()
    routing_executor.add_node(routing_monitor)
    try:
        routing_executor.spin()
    except KeyboardInterrupt:
        pass
    routing_executor.remove_node(routing_monitor)
    routing_monitor.destroy_node()
    rclpy.shutdown()