'''
Package:   carla
Filename:  liaison_node.py
Author:    Will Heitman (w at heit.mn)

This node handles our ROS-based communication with the
CARLA Leaderboard. This includes publishing to /carla/hero/status,
but more functionality will be added in the future.
'''

import os
import rclpy
import random
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
import time

from carla_msgs.msg import CarlaRoute, CarlaWorldInfo, CarlaEgoVehicleStatus
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from nova_msgs.msg import PedalPosition, SteeringPosition
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

import carla


class LeaderboardLiaisonNode(Node):

    def __init__(self):
        # self.get_logger().info("woo")
        super().__init__('liaison_node')
        self.status_pub = self.create_publisher(
            Bool, '/carla/hero/status', qos_profile=QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        status_timer = self.create_timer(1.0, self.publish_hero_status)

        self.waypoint_sub = self.create_subscription(
            CarlaRoute, '/carla/hero/global_plan', self.route_cb, 10)
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        self.wp_marker_pub = self.create_publisher(
            MarkerArray, '/viz/wayppoints', 10)

        self.world_info_sub = self.create_subscription(
            CarlaWorldInfo, '/carla/world_info', self.world_info_cb, 10)
        self.world_info_pub = self.create_publisher(
            CarlaWorldInfo, '/carla/world_info', 10)

        self.route_path_pub = self.create_publisher(
            Path, '/route/rough_path', 10)
        self.route_repub_timer = self.create_timer(1.0, self.publish_route)

        self.world_info_cached = None
        self.world_info_repub_timer = self.create_timer(
            5.0, self.repub_world_info)

        vehicle_status_sub = self.create_subscription(
            CarlaEgoVehicleStatus, '/carla/hero/vehicle_status', self.vehicleStatusCb, 1)
        self.steering_pos_pub = self.create_publisher(
            SteeringPosition, '/steering_pos', 1)
        self.throttle_pos_pub = self.create_publisher(
            PedalPosition, '/throttle_pos', 1)
        self.brake_pos_pub = self.create_publisher(
            PedalPosition, '/brake_pos', 1)

        self.route = None
        self.clock = Clock()

        connect_to_carla = True  # TODO: Make this a param

        if connect_to_carla is False:
            return

        self.client = carla.Client(
            'localhost', 2000 + int(os.environ['ROS_DOMAIN_ID']))
        self.client.set_timeout(60)
        self.world = self.client.get_world()
        blueprint_library = self.world.get_blueprint_library()

        WALKER_COUNT = 0
        CAR_COUNT = 60

        # Wait for ego to spawn
        time.sleep(5.0)

        # Find ego vehicle
        actor_list = self.world.get_actors()
        self.ego = actor_list.filter("vehicle.tesla.model3")[0]
        self.get_logger().info(str(self.ego))

        self.actor_true_pose_timer = self.create_timer(
            0.1, self.publish_true_pose)
        self.true_pose_pub = self.create_publisher(
            PoseStamped, '/true_pose', 10)

        # Spawn some cars
        car_count = 0
        car_spawns = self.world.get_map().get_spawn_points()
        for spawn in car_spawns:
            if car_count > CAR_COUNT:
                continue
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*.*'))
            actor = self.world.try_spawn_actor(vehicle_bp, spawn)
            if actor is not None:
                actor.set_autopilot(True)
                car_count += 1

        self.get_logger().info(f"Spawned {car_count} cars!")

        # Spawn some walkers (pedestrians)
        walker_ai_bp = blueprint_library.filter('controller.ai.walker')[0]
        walker_count = 0
        while walker_count < WALKER_COUNT:
            if walker_count > 30:
                continue
            walker_bp = random.choice(blueprint_library.filter('walker.*.*'))
            spawn_point = carla.Transform()
            spawn_point.location = self.world.get_random_location_from_navigation()
            actor = self.world.try_spawn_actor(
                walker_bp, spawn_point)
            if actor is not None:
                ai_controller = self.world.try_spawn_actor(
                    walker_ai_bp, spawn_point, attach_to=actor)
                if ai_controller is not None:
                    ai_controller.go_to_location(
                        self.world.get_random_location_from_navigation())
                walker_count += 1

    def vehicleStatusCb(self, msg: CarlaEgoVehicleStatus):

        steering_pos = msg.control.steer
        throttle_pos = msg.control.throttle
        brake_pos = msg.control.brake

        throttle_msg = PedalPosition()
        throttle_msg.data = throttle_pos

        brake_msg = PedalPosition()
        brake_msg.data = brake_pos

        steering_msg = SteeringPosition()
        steering_msg.data = steering_pos

        self.throttle_pos_pub.publish(throttle_msg)
        self.brake_pos_pub.publish(brake_msg)
        self.steering_pos_pub.publish(steering_msg)

    def publish_true_pose(self):
        carla_tf = self.ego.get_transform()
        carla_pos = carla_tf.location
        msg = PoseStamped()
        msg.pose.position.x = carla_pos.x
        msg.pose.position.y = carla_pos.y
        msg.pose.position.z = carla_pos.z
        msg.header.frame_id = 'map'
        msg.header.stamp = self.clock.clock

        self.true_pose_pub.publish(msg)

    def set_all_lights_to_green(self):
        actors = self.world.get_actors()
        print(actors)
        for actor in actors:
            if actor.type_id == 'traffic.traffic_light':
                actor.set_red_time(1.0)
                actor.set_yellow_time(0.5)

    def world_info_cb(self, msg: CarlaWorldInfo):
        if msg.opendrive == "":
            return
        self.world_info_cached = msg

    def repub_world_info(self):
        if self.world_info_cached is None:
            return
        self.world_info_pub.publish(self.world_info_cached)

    def clock_cb(self, msg: Clock):
        self.clock = msg

    def publish_route(self):

        if self.route is None:
            return  # Route not yet received

        if self.clock is None:
            return  # Clock not yet received

        # Publish the route as a path
        path = Path()

        for pose in self.route.poses:
            pose: Pose
            pose.position.z = 0.0  # Ignore height. TODO: Support z != 0.
            pose_stamped = PoseStamped()
            pose_stamped.pose = pose
            pose_stamped.header.frame_id = 'map'
            # TODO: Set each pose stamp properly
            pose_stamped.header.stamp = self.clock.clock
            path.poses.append(pose_stamped)

        path.header.frame_id = 'map'
        path.header.stamp = self.clock.clock

        self.route_path_pub.publish(path)

    def route_cb(self, msg: CarlaRoute):
        self.get_logger().info("Received route with {} waypoints".format(len(msg.poses)))
        self.route = msg

        # wp_marker_array = MarkerArray()
        # wp_marker = Marker()
        # wp_marker.header.frame_id = 'map'
        # wp_marker.header.stamp = self.clock.clock
        # wp_marker.frame_locked = True
        # wp_marker.ns = 'waypoints'
        # wp_marker.type = Marker.ARROW
        # wp_marker.action = Marker.ADD
        # marker_color = ColorRGBA()
        # marker_color.r, marker_color.g, marker_color.b, marker_color.a = (
        #     1.0, 1.0, 1.0, 10)

        # wp_marker.color = marker_color
        # wp_marker.scale.x = 5.0
        # wp_marker.scale.y = 2.0
        # wp_marker.scale.z = 2.0

        # for pose in msg.poses:
        #     wp_marker.pose = pose
        #     wp_marker_array.markers.append(wp_marker)

        # self.wp_marker_pub.publish(wp_marker_array)

    def publish_hero_status(self):
        """Publish a true Bool to the leaderboard status topic"""
        status = Bool()
        status.data = True  # This means "good to go!"
        self.status_pub.publish(status)

        # list_actor = self.world.get_actors()
        # for actor_ in list_actor:
        #     if isinstance(actor_, carla.TrafficLight):
        #         # for any light, first set the light state, then set time. for yellow it is
        #         # carla.TrafficLightState.Yellow and Red it is carla.TrafficLightState.Red
        #         actor_.set_state(carla.TrafficLightState.Green)
        #         actor_.set_green_time(1000.0)
        #         # actor_.set_green_time(5000.0)
        #         # actor_.set_yellow_time(1000.0)


def main(args=None):
    rclpy.init(args=args)

    liaison_node = LeaderboardLiaisonNode()

    rclpy.spin(liaison_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    liaison_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
