#!/usr/bin/python

'''
Nova at UT Dallas, 2022

The Navigator Simulation Bridge for CARLA

The goal is to mimick Hail Bopp as much as possible.

Targetted sensors:
- GNSS (GPS)
✓ IMU ()
- Front and rear Lidar
✓ Front  RGB camera
✓ Front depth camera
- CARLA ground truths for
    - Detected objects
    ✓ Car's odometry (position, orientation, speed)
    ✓ CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

Todos:
- Specific todos are dispersed in this script. General ones are here.
- Ensure all sensors publish in ROS coordinate system, NOT Unreal Engine's.

'''

# GLOBAL CONSTANTS
# TODO: Move to ROS param file, read on init. WSH.
CLIENT_PORT = 2000
CLIENT_WORLD = 'Town10HD'
EGO_AUTOPILOT_ENABLED = True
EGO_MODEL = 'vehicle.audi.etron'
GNSS_PERIOD = 1/(2.0) # 2 Hz
GROUND_TRUTH_ODOM_PERIOD = 1/(10.0) # 10 Hz
LIDAR_PERIOD = 1/(10.0) # 20 Hz
OBSTACLE_QTY_CAR = 10 # Spawn n cars
OBSTACLE_QTY_PED = 10 # Spawn n peds

import carla
import random
import rclpy
from rclpy.node import Node

# For lidar data manipulation/conversion. WSH.
import ros2_numpy as rnp
import numpy as np

# For camera data conversion. WSH.
from cv_bridge import CvBridge

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener
import math
from os.path import exists

# Message definitons
from std_msgs.msg import Header
from sensor_msgs.msg import Image # For cameras
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry # For GPS, ground truth
from voltron_msgs.msg import PeddlePosition, SteeringPosition
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from scipy.spatial.transform import Rotation as R

from sensor_msgs.msg import PointCloud2

class SimBridgeNode(Node):

    def get_color_image(self, carla_image: carla.Image, encoding: str = 'bgra8'): # Straight-up copied from carla_ros_bridge. WSH.
        """
        Function to transform the received carla camera data into a ROS image message
        """
        carla_image_data_array = np.ndarray(
        shape=(carla_image.height, carla_image.width, 4),
        dtype=np.uint8, buffer=carla_image.raw_data)
        img_msg = self.cv_bridge.cv2_to_imgmsg(carla_image_data_array, encoding=encoding)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = '/base_link'

        return img_msg

    def get_depth_image(self, carla_image: carla.Image, encoding: str = 'passthrough'): # Straight-up copied from carla_ros_bridge. WSH.
        bgra_image = np.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=np.uint8, buffer=carla_image.raw_data)

        # Apply (R + G * 256 + B * 256 * 256) / (256**3 - 1) * 1000
        # according to the documentation:
        # https://carla.readthedocs.io/en/latest/cameras_and_sensors/#camera-depth-map
        scales = np.array([65536.0, 256.0, 1.0, 0]) / (256**3 - 1) * 1000
        depth_image = np.dot(bgra_image, scales).astype(np.float32)

        img_msg = self.cv_bridge.cv2_to_imgmsg(depth_image, encoding=encoding)
        # the camera data is in respect to the camera's own frame
        # img_msg.header = self.get_msg_header(timestamp=carla_camera_data.timestamp)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = '/base_link'

        return img_msg

    def birds_eye_cam_cb(self, data: carla.Image):
        img_msg = self.get_color_image(data)
        self.birds_eye_cam_pub.publish(img_msg)

    # TODO: Fix so that full scan is registered each time. WSH.
    def front_lidar_cb(self, data: carla.LidarMeasurement):
        # Taken from carla_ros_bridge's "lidar.py". WSH.
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='lidar_front'
        )

        lidar_data = np.fromstring(
            bytes(data.raw_data), dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        lidar_data.dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ]
        # we take the opposite of y axis
        # (as lidar point are express in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1

        msg = rnp.msgify(PointCloud2, lidar_data)
        msg.header = header
        self.front_lidar_pub.publish(msg)
        # point_cloud_msg = create_cloud(header, fields, lidar_data)
        # self.lidar_publisher.publish(point_cloud_msg)

    def front_rgb_cb(self, data: carla.Image):
        img_msg = self.get_color_image(data, encoding='bgra8')
        self.front_rgb_pub.publish(img_msg)

    def front_depth_cb(self, data: carla.Image):
        img_msg = self.get_depth_image(data)
        self.front_depth_pub.publish(img_msg)

    def primary_imu_cb(self, data: carla.IMUMeasurement):
        imu_msg = Imu()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = '/base_link' # TODO add custom tf frame. WSH.
        # TODO convert coordinate frame from Unreal Engine's to ROS's. WSH.
        imu_msg.linear_acceleration.x = data.accelerometer.x
        imu_msg.linear_acceleration.y = data.accelerometer.y
        imu_msg.linear_acceleration.z = data.accelerometer.z
        imu_msg.angular_velocity.x = data.gyroscope.x
        imu_msg.angular_velocity.y = data.gyroscope.y
        imu_msg.angular_velocity.z = data.gyroscope.z

        self.primary_imu_pub.publish(imu_msg)

    def true_odom_cb(self):
        ego: carla.Actor = self.ego

        odom = Odometry()
        odom.header.frame_id = 'base_link'
        odom.header.stamp = self.get_clock().now().to_msg()

        ego_position = Point()
        ego_orientation = Quaternion()
        ego_tf: carla.Transform = ego.get_transform()
        ego_position.x = ego_tf.location.x
        ego_position.y = ego_tf.location.y
        ego_position.z = ego_tf.location.z

        ego_quat = R.from_euler(
            'yzx',
            [ego_tf.rotation.pitch,
            ego_tf.rotation.yaw,
            ego_tf.rotation.roll]
        ).as_quat()
        ego_orientation.x = ego_quat[0]
        ego_orientation.y = ego_quat[1]
        ego_orientation.z = ego_quat[2]
        ego_orientation.w = ego_quat[3]

        odom.pose.pose.position = ego_position
        odom.pose.pose.orientation = ego_orientation

        self.ground_truth_odom_pub.publish(odom)

        # Now add some very simple noise to fake a GPS sensor. WSH.
        # We'll go easy and add random deviation of just <2m on each axis
        ego_position.x += (random.randrange(-200, 200)/100.0)
        ego_position.y += (random.randrange(-200, 200)/100.0)
        ego_position.z += (random.randrange(-200, 200)/100.0)
        # Similarly, add <0.175 radians (~10 degrees) random to yaw
        ego_tf.rotation.yaw += random.randrange(-175, 175)/1000.0
        ego_quat = R.from_euler(
            'yzx',
            [ego_tf.rotation.pitch,
            ego_tf.rotation.yaw,
            ego_tf.rotation.roll]
        ).as_quat()
        odom.pose.pose.position = ego_position
        odom.pose.pose.orientation = ego_orientation
        self.gnss_pub.publish(odom)

    def __init__(self):
        super().__init__('sim_bridge_node')

        # Define sensors
        self.front_lidar: carla.ServerSideSensor
        self.front_lidar_cloud = np.array([])
        self.cv_bridge = CvBridge()

        # Create our publishers
        self.birds_eye_cam_pub = self.create_publisher(
            Image,
            '/carla/birds_eye_rgb',
            10
        )

        self.front_depth_pub = self.create_publisher(
            Image,
            '/camera_front/depth',
            10
        )

        self.front_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar_front/points_raw',
            10
        )

        self.front_rgb_pub = self.create_publisher(
            Image,
            '/camera_front/rgb',
            10
        )

        self.rear_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar_rear/points_raw',
            10
        )

        self.gnss_pub = self.create_publisher(
            Odometry,
            '/gnss/odom',
            10
        )

        self.ground_truth_odom_pub = self.create_publisher(
            Odometry,
            '/carla/odom',
            10
        )

        self.primary_imu_pub = self.create_publisher(
            Imu,
            '/imu_primary/data',
            10
        )

        self.rgb_front_pub = self.create_publisher(
            Image,
            '/camera_front/rgb',
            10
        )

        self.steering_command_sub = self.create_subscription(
            SteeringPosition,
            '/command/steering_position',
            self.steering_command_cb,
            10
        )

        self.ground_truth_odom_timer = self.create_timer(
            GROUND_TRUTH_ODOM_PERIOD, self.true_odom_cb
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.connect_to_carla()

    def connect_to_carla(self):
        # Connect to client, load world
        self.get_logger().info("Connecting to CARLA on port {}".format(CLIENT_PORT))
        client = carla.Client('localhost', CLIENT_PORT)
        client.set_timeout(20.0)
        self.world = client.load_world(CLIENT_WORLD)

        # Spawn ego vehicle
        # Get car blueprint
        self.blueprint_library = self.world.get_blueprint_library()
        # for bp in blueprint_library.filter('vehicle.*.*'):
        #     self.get_logger().info("{}".format(bp.id))
        vehicle_bp = self.blueprint_library.find(EGO_MODEL)

        # collision_sensor_bp = blueprint_library.find('sensor.other.collision')
        # Get random spawn point
        random_spawn = self.world.get_map().get_spawn_points()[0]
        self.get_logger().info("Spawning ego vehicle ({}) @ {}".format(EGO_MODEL, random_spawn))
        self.ego: carla.Vehicle = self.world.spawn_actor(vehicle_bp, random_spawn) 
        # TODO: Destroy ego actor when node exits or crashes. Currently keeps actor alive in CARLA,
        # which eventually leads to memory overflow. WSH.
        self.ego.set_autopilot(enabled=EGO_AUTOPILOT_ENABLED)

        self.add_ego_sensors()        

    def add_ego_sensors(self):
        # Attach Lidar sensor
        lidar_bp = self.blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '16') # VLP-16
        lidar_bp.set_attribute('sensor_tick', str(LIDAR_PERIOD))
        lidar_bp.set_attribute('rotation_frequency', '40')
        # lidar_bp.set_attribute('points_per_second', '11,200')
        relative_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation()) # TODO: Fix transform
        self.front_lidar = self.world.spawn_actor(lidar_bp, relative_transform, attach_to=self.ego)
        self.front_lidar.listen(self.front_lidar_cb)

        # Attach GNSS sensor TODO (WSH)
        # gnss_bp = self.blueprint_library.find('sensor.other.gnss')
        # gnss_bp.set_attribute('noise_alt_stddev', '0.0') # We can add noise here. TODO.
        # gnss_bp.set_attribute('noise_lat_stddev', '0.0')
        # gnss_bp.set_attribute('noise_lon_stddev', '0.0')
        # gnss_bp.set_attribute('sensor_tick', str(GNSS_PERIOD))
        # relative_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation()) # TODO: Fix transform
        # self.gnss = self.world.spawn_actor(lidar_bp, relative_transform, attach_to=ego)
        # self.gnss.listen(self.gnss_cb)

        # Attach bird's-eye camera
        birds_eye_cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        birds_eye_cam_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=20.0), carla.Rotation(pitch=-90.0))
        self.birds_eye_cam = self.world.spawn_actor(birds_eye_cam_bp, relative_transform, attach_to=self.ego)
        self.birds_eye_cam.listen(self.birds_eye_cam_cb)

        # Attach front rgb camera
        front_rgb_bp = self.blueprint_library.find('sensor.camera.rgb')
        front_rgb_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.front_rgb = self.world.spawn_actor(front_rgb_bp, relative_transform, attach_to=self.ego)
        self.front_rgb.listen(self.front_rgb_cb)

        # Attach front depth camera
        front_depth_bp = self.blueprint_library.find('sensor.camera.depth')
        front_depth_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.front_depth = self.world.spawn_actor(front_depth_bp, relative_transform, attach_to=self.ego)
        self.front_depth.listen(self.front_depth_cb)

        # Attach front camera's IMU
        primary_imu_bp = self.blueprint_library.find('sensor.other.imu')
        primary_imu_bp.set_attribute('sensor_tick', str(0.1))
        # TODO: Add covariance. WSH.
        relative_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.primary_imu = self.world.spawn_actor(primary_imu_bp, relative_transform, attach_to=self.ego)
        self.primary_imu.listen(self.primary_imu_cb)

    def spawn_obstacles(self):
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(OBSTACLE_QTY_PED):
            self.world.SpawnActor(walker_controller_bp, carla.Transform(), parent_walker)

def main(args=None):
    rclpy.init(args=args)

    sim_bridge_node = SimBridgeNode()

    rclpy.spin(sim_bridge_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    sim_bridge_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()