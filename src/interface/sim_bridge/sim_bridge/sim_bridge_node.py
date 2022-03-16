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

import random

# GLOBAL CONSTANTS
# TODO: Move to ROS param file, read on init. WSH.
CLIENT_PORT = 2000
CLIENT_WORLD = 'Town10HD'
EGO_AUTOPILOT_ENABLED = False
EGO_MODEL = 'vehicle.audi.etron'
GNSS_PERIOD = 1/(2.0) # 2 Hz
GROUND_TRUTH_OBJ_PERIOD = 1/(2.0) # 2 Hz (purposely bad)
GROUND_TRUTH_ODOM_PERIOD = 1/(10.0) # 10 Hz
LIDAR_PERIOD = 1/(10.0) # 10 Hz
SPEEDOMETER_PERIOD = 1/(10.0) # 10 Hz
OBSTACLE_QTY_VEHICLE = 0 # Spawn n cars
OBSTACLE_QTY_PED = 0 # Spawn n peds

# Map-specific constants
MAP_ORIGIN_LAT = 0.0 # degrees
MAP_ORIGIN_LON = 0.0 # degrees

# Sensor noise constants
M_TO_DEG = 9e-6 # APPROXIMATE! WSH.

GNSS_ALT_BIAS = random.uniform(0.25,1.0)*M_TO_DEG # Degrees -  https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor
GNSS_ALT_SDEV = 0.5*M_TO_DEG
GNSS_LAT_BIAS = random.uniform(0.25,1.0)*M_TO_DEG
GNSS_LAT_SDEV = 0.5*M_TO_DEG
GNSS_LON_BIAS = random.uniform(0.25,1.0)*M_TO_DEG
GNSS_LON_SDEV = 0.5*M_TO_DEG

# Publish a true map->base_link transform. Disable this if
# another localization algorithm (ukf, ndt, etc.) is running! WSH.
PULBISH_MAP_BL_TRANSFORM = False 

import sys
sys.path.append('/home/share/carla/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg')

import carla

import rclpy
from rclpy.node import Node

# For lidar data manipulation/conversion. WSH.
import ros2_numpy as rnp
import numpy as np

# For camera data conversion. WSH.
from cv_bridge import CvBridge

# For lat/lon-> ENU conversions. WSH.
import pymap3d

from tf2_ros import TransformException, TransformStamped
import tf2_msgs
from tf2_ros.buffer import Buffer
import tf2_py
from tf2_ros.transform_listener import TransformListener
from tf2_ros.transform_broadcaster import TransformBroadcaster
import math
from os.path import exists

# Message definitons
from std_msgs.msg import Bool
from std_msgs.msg import Header
from sensor_msgs.msg import Image # For cameras
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry # For GPS, ground truth
from voltron_msgs.msg import PeddlePosition, SteeringPosition, Obstacle3DArray, Obstacle3D, BoundingBox3D
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, TwistWithCovarianceStamped

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

    def gnss_cb(self, data: carla.GnssMeasurement) :
        msg = Odometry()
        posewithcov = PoseWithCovariance()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'gnss'

        enu_coords = pymap3d.geodetic2enu(
            data.latitude, data.longitude, 0.0,
            MAP_ORIGIN_LAT, MAP_ORIGIN_LON, 0.0
        )

        posewithcov.pose.position.x = enu_coords[0]
        posewithcov.pose.position.y = enu_coords[1]
        posewithcov.pose.position.z = enu_coords[2] # This should be 0.0. We don't care about altitude. WSH.
        
        ego_tf = self.ego.get_transform()
        ego_quat = R.from_euler(
            'yzx',
            [ego_tf.rotation.pitch*-1*math.pi/180.0,
            ego_tf.rotation.yaw*-1*math.pi/180.0,
            ego_tf.rotation.roll*-1*math.pi/180.0]
        ).as_quat()
        posewithcov.pose.orientation.x = ego_quat[0]
        posewithcov.pose.orientation.y = ego_quat[1]
        posewithcov.pose.orientation.z = ego_quat[2]
        posewithcov.pose.orientation.w = ego_quat[3]

        ego_vel = self.ego.get_velocity()
        twist_linear = Vector3()
        twist_linear.x = ego_vel.x
        twist_linear.y = ego_vel.y * -1
        twist_linear.z = ego_vel.z
        msg.twist.twist.linear = twist_linear

        accuracy = 1.0 # Meters, s.t. pos.x = n +/- accuracy

        posewithcov.covariance = [1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 1.0]
                                  

        msg.pose = posewithcov

        self.gnss_pub.publish(msg)

    def primary_imu_cb(self, data: carla.IMUMeasurement):
        imu_msg = Imu()

        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu'
        # UE4 uses a left-handed system, so we must invert y axis and angles. WSH
        imu_msg.linear_acceleration.x = data.accelerometer.x
        imu_msg.linear_acceleration.y = data.accelerometer.y * -1
        imu_msg.linear_acceleration.z = data.accelerometer.z * -1
        imu_msg.angular_velocity.x = data.gyroscope.x * -1 
        imu_msg.angular_velocity.y = data.gyroscope.y * -1
        imu_msg.angular_velocity.z = data.gyroscope.z * -1
        imu_msg.linear_acceleration_covariance = [0.3, 0.0, 0.0,
                                                  0.0, 0.3, 0.0,
                                                  0.0, 0.0, 0.3]

        # imu_msg.angular_velocity_covariance   =  [0.1, 0.0, 0.0,
        #                                           0.0, 0.1, 0.0,
        #                                           0.0, 0.0, 0.1]

        self.primary_imu_pub.publish(imu_msg)
    
    def process_command(self):
        cmd = carla.VehicleControl()
        cmd.throttle = self.throttle_cmd
        cmd.brake = self.brake_cmd
        cmd.steer = self.steering_cmd
        cmd.reverse = self.reverse_cmd
        vel = self.ego.get_velocity()
        speed = math.floor(math.sqrt((vel.x ** 2) + (vel.y ** 2) + (vel.z ** 2))/0.447) * 0.447
        if (speed > 10.2):
            cmd.throttle = 0.0 # Cap our speed at 23 mph. WSH.
            # self.get_logger().warn("Your speed of {} has maxed out".format(speed))
        self.ego.apply_control(cmd)

    def sem_lidar_cb(self, data: carla.SemanticLidarMeasurement):
        # For output classes, see https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
        # Roads: yellow
        # Other ground: green
        # Vehicles (incl. bikes): purple
        # Signs: blue
        # Other: off-white
        road_color_rgb = [252,255,166] # FCFFA6, yellow
        ground_color_rgb = [193,255,215] # C1FFD7, light green
        vehicle_color_rgb = [202,184,255] # CAB8FF, purple
        sign_color_rgb = [181,222,255] # B5DEFF, light blue
        other_color_rgb = [249,248,248] # F9F8F8, off-white

        # for det in data:
        #     self.get_logger().info("{}".format(det))

        # Taken from carla_ros_bridge's "lidar.py". WSH.
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='lidar_front'
        )

        lidar_data = np.fromstring(
            bytes(data.raw_data), dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 6), 6))
        lidar_data.dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('angle', np.float32),
            ('id', np.int32),
            ('intensity', np.int32)
        ]
        # we take the opposite of y axis
        # (as lidar point are expressed in left handed coordinate system, and ros need right handed)
        lidar_data['y'] *= -1

        msg = rnp.msgify(PointCloud2, lidar_data)
        msg.header = header
        self.sem_lidar_pub.publish(msg)

        lidar_data = lidar_data[lidar_data['intensity'] == 7.0]
        msg = rnp.msgify(PointCloud2, lidar_data)
        msg.header = header
        self.sem_road_lidar_pub.publish(msg)

        # for detection in data:
        #     self.get_logger().info("{}".format(detection))

    def steering_command_cb(self, msg: SteeringPosition):
        self.steering_cmd = msg.data

    def throttle_command_cb(self, msg: PeddlePosition):
        self.throttle_cmd = msg.data

    def brake_command_cb(self, msg: PeddlePosition):
        self.brake_cmd = msg.data

    def reverse_command_cb(self, msg: Bool):
        self.reverse_cmd = msg.data

    def publish_true_boxes(self):
        
        vehicles = self.world.get_actors().filter('vehicle.*')
        # self.get_logger().info("Publishing {} boxes.".format(len(vehicles)))

        obstacles = []
        for vehicle in vehicles:
            obst = Obstacle3D()
            obst.id = vehicle.id
            obst.label = obst.CAR # TODO: Generalize, e.g. "bike", "car"
            obst.confidence = random.uniform(0.5, 1.0)

            bbox = vehicle.bounding_box

            # Set velocity
            actor_vel = vehicle.get_velocity()
            obst.velocity.x = actor_vel.x
            obst.velocity.y = actor_vel.y*-1 # Fix coordinate system
            obst.velocity.z = actor_vel.z

            # Set bounding box
            pos = Point()
            pos.x = bbox.location.x
            pos.y = bbox.location.y*-1
            pos.z = bbox.location.z

            actor_tf: carla.Transform = vehicle.get_transform()

            actor_quat = R.from_euler(
            'yzx',
                [actor_tf.rotation.pitch*-1*math.pi/180.0,
                actor_tf.rotation.yaw*-1*math.pi/180.0-math.pi,
                actor_tf.rotation.roll*-1*math.pi/180.0]
            ).as_quat()
            orientation_msg = Quaternion()
            orientation_msg.x = actor_quat[0]
            orientation_msg.y = actor_quat[1]
            orientation_msg.z = actor_quat[2]
            orientation_msg.w = actor_quat[3]

            obst.bounding_box.center.position = pos
            obst.bounding_box.center.orientation = orientation_msg
            obst.bounding_box.size = Vector3(
                x = bbox.extent.x,
                y = bbox.extent.y,
                z = bbox.extent.z
            )

            obstacles.append(obst)
        obstacles_msg = Obstacle3DArray()
        obstacles_msg.obstacles = obstacles
        obstacles_msg.header.stamp = self.get_clock().now().to_msg()
        obstacles_msg.header.frame_id = 'map'

        self.ground_truth_obst_pub.publish(obstacles_msg)
        # self.get_logger().info("{}".format(obstacles))

    def pub_speedometer(self):
        vel = self.ego.get_velocity()
        twist_linear = TwistWithCovarianceStamped()
        speed = math.floor(math.sqrt((vel.x ** 2) + (vel.y ** 2) + (vel.z ** 2))/0.447) * 0.447
        twist_linear.twist.twist.linear.x = speed
        twist_linear.twist.covariance[0] = 0.25 # Our speedometer is accurate to ~0.5 m/s. Covariance is the square of stdev.
        twist_linear.header.frame_id = 'base_link'
        twist_linear.header.stamp = self.get_clock().now().to_msg()

        self.speedometer_pub.publish(twist_linear)

    def true_odom_cb(self):
        ego: carla.Actor = self.ego

        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.child_frame_id = 'base_link'

        ego_position = Point()
        ego_orientation = Quaternion()
        ego_tf: carla.Transform = ego.get_transform()
        ego_position.x = ego_tf.location.x
        ego_position.y = ego_tf.location.y*-1
        ego_position.z = 0.0 # Force to zero

        ego_quat = R.from_euler(
            'yzx',
            [ego_tf.rotation.pitch*-1*math.pi/180.0,
            ego_tf.rotation.yaw*-1*math.pi/180.0-math.pi,
            ego_tf.rotation.roll*-1*math.pi/180.0]
        ).as_quat()
        ego_orientation.x = ego_quat[0]
        ego_orientation.y = ego_quat[1]
        ego_orientation.z = ego_quat[2]
        ego_orientation.w = ego_quat[3]

        odom.pose.pose.position = ego_position
        odom.pose.pose.orientation = ego_orientation

        self.ground_truth_odom_pub.publish(odom)

        # Publish tf if enabled
        if PULBISH_MAP_BL_TRANSFORM:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = odom.child_frame_id
            translation = Vector3(
                x = odom.pose.pose.position.x,
                y = odom.pose.pose.position.y,
                z = odom.pose.pose.position.z
            )
            t.transform.translation = translation
            t.transform.rotation = Quaternion(
                x = ego_quat[0],
                y = ego_quat[1],
                z = ego_quat[2],
                w = ego_quat[3]
            )
            self.tf_broadcaster.sendTransform(t)

    def __init__(self):
        super().__init__('sim_bridge_node')

        # Define sensors
        self.front_lidar: carla.ServerSideSensor
        self.front_lidar_cloud = np.array([])
        self.cv_bridge = CvBridge()

        # Define vehicle command state
        self.steering_cmd = 0.0
        self.brake_cmd = 0.0
        self.throttle_cmd = 0.0
        self.reverse_cmd = False

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

        self.ground_truth_obst_pub = self.create_publisher(
            Obstacle3DArray,
            '/objects',
            10
        )

        self.primary_imu_pub = self.create_publisher(
            Imu,
            '/imu_primary/data',
            10
        )

        self.speedometer_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/can/speedometer_twist',
            10
        )

        self.sem_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar/semantic',
            10
        )

        self.sem_road_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar/semantic/road',
            10
        )

        self.steering_command_sub = self.create_subscription(
            SteeringPosition,
            '/command/steering_position',
            self.steering_command_cb,
            10
        )

        self.throttle_command_sub = self.create_subscription(
            PeddlePosition,
            '/command/throttle_position',
            self.throttle_command_cb,
            10
        )

        self.brake_command_sub = self.create_subscription(
            PeddlePosition,
            '/command/brake_position',
            self.brake_command_cb,
            10
        )

        self.reverse_command_sub = self.create_subscription(
            Bool,
            '/command/reverse',
            self.reverse_command_cb,
            10
        )

        self.ground_truth_odom_timer = self.create_timer(
            GROUND_TRUTH_ODOM_PERIOD, self.true_odom_cb
        )

        self.command_timer = self.create_timer(0.1, self.process_command)

        self.ground_truth_objects_timer = self.create_timer(
            GROUND_TRUTH_OBJ_PERIOD, self.publish_true_boxes
        )

        self.speedometer_timer = self.create_timer(
            SPEEDOMETER_PERIOD, self.pub_speedometer
        )
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_to_carla()

    def add_vehicles(self, vehicle_count: int):
        self.get_logger().info("Spawning {} vehicles".format(vehicle_count))

        for vehicle in range(vehicle_count):
            # Choose a vehicle blueprint at random.
            vehicle_bp = random.choice(self.blueprint_library.filter('vehicle.*.*'))
            random_spawn = random.choice(self.world.get_map().get_spawn_points())
            self.get_logger().info("Spawning vehicle ({}) @ {}".format(vehicle_bp.id, random_spawn))
            vehicle = self.world.try_spawn_actor(vehicle_bp, random_spawn)
            if vehicle is not None:
                vehicle.set_autopilot(enabled=True)

    def add_pedestrians(self, count: int):
        self.get_logger().info("Spawning {} pedestrians".format(count))

        for ped in range(count):
            # Choose a vehicle blueprint at random.
            ped_bp = random.choice(self.blueprint_library.filter('walker.*.*'))
            spawn = self.world.get_random_location_from_navigation()
            self.get_logger().info("Spawning ped ({}) @ {}".format(ped_bp.id, spawn))
            random_spawn = random.choice(self.world.get_map().get_spawn_points())
            random_spawn.location = spawn
            ped = self.world.try_spawn_actor(ped_bp, random_spawn)
            # if ped is not None:
            #     ped.set_autopilot(enabled=True)

    def connect_to_carla(self):
        # Connect to client, load world
        self.get_logger().info("Connecting to CARLA on port {}".format(CLIENT_PORT))
        client = carla.Client('localhost', CLIENT_PORT)
        client.set_timeout(20.0)
        self.world = client.load_world(CLIENT_WORLD)
        
        # Forcefully destroy existing actors
        # if len(self.world.get_actors()) > 0:
        #     self.get_logger().info("Removing {} old actors".format(len(self.world.get_actors())))
        #     for actor in self.world.get_actors():
        #         actor.destroy()

        # Spawn ego vehicle
        # Get car blueprint
        self.blueprint_library = self.world.get_blueprint_library()

        # collision_sensor_bp = blueprint_library.find('sensor.other.collision')
        # Get random spawn point
        # random_spawn = self.world.get_map().get_spawn_points()[0]
        spawn_loc = carla.Location()
        spawn_loc.x = 0.0
        spawn_loc.y = 29.0
        spawn_loc.z = 2.0 # Start up in the air
        spawn = carla.Transform()
        spawn.location = spawn_loc
        self.get_logger().info("Spawning ego vehicle ({}) @ {}".format(EGO_MODEL, spawn))
        vehicle_bp = self.blueprint_library.find(EGO_MODEL)
        self.ego: carla.Vehicle = self.world.spawn_actor(vehicle_bp, spawn) 
        # TODO: Destroy ego actor when node exits or crashes. Currently keeps actor alive in CARLA,
        # which eventually leads to memory overflow. WSH.
        self.ego.set_autopilot(enabled=EGO_AUTOPILOT_ENABLED)

        self.add_ego_sensors()        

        self.add_vehicles(OBSTACLE_QTY_VEHICLE)

        self.add_pedestrians(OBSTACLE_QTY_PED)

    def add_ego_sensors(self):

        front_lidar_tf = carla.Transform(
            carla.Location(x = 3.4, y = 0.902, z = 0.7876),
            carla.Rotation(roll=0.85943669, pitch = 1.71887339, yaw = 83.0788803) # DEGREES, left-handed. Come on, CARLA...
        ) 

        # Attach Lidar sensor
        lidar_bp = self.blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '16') # VLP-16
        lidar_bp.set_attribute('sensor_tick', str(LIDAR_PERIOD))
        lidar_bp.set_attribute('rotation_frequency', '40')
        # lidar_bp.set_attribute('points_per_second', '11,200')
        relative_transform = front_lidar_tf
        self.front_lidar = self.world.spawn_actor(lidar_bp, relative_transform, attach_to=self.ego)
        self.front_lidar.listen(self.front_lidar_cb)

        # Semantic lidar
        sem_lidar_bp = self.blueprint_library.find('sensor.lidar.ray_cast_semantic')
        sem_lidar_bp.set_attribute('channels', '16') # VLP-16
        sem_lidar_bp.set_attribute('rotation_frequency','30') # "30" is CARLA's default FPS.
        # sem_lidar_bp.set_attribute('sensor_tick', str(LIDAR_PERIOD))
        # sem_lidar_bp.set_attribute('rotation_frequency', '40')
        # sem_lidar_bp.set_attribute('points_per_second', '11,200') # 3.4 -0.902 0.7876
        relative_transform = front_lidar_tf
        self.sem_lidar = self.world.spawn_actor(sem_lidar_bp, relative_transform, attach_to=self.ego)
        self.sem_lidar.listen(self.sem_lidar_cb)

        # Attach GNSS sensor
        gnss_bp = self.blueprint_library.find('sensor.other.gnss')
        gnss_bp.set_attribute('noise_alt_stddev', str(GNSS_ALT_SDEV)) # We can add noise here.
        gnss_bp.set_attribute('noise_alt_bias', str(GNSS_ALT_BIAS)) 
        gnss_bp.set_attribute('noise_lat_stddev', str(GNSS_LAT_SDEV))
        gnss_bp.set_attribute('noise_lat_bias', str(GNSS_LAT_BIAS)) 
        gnss_bp.set_attribute('noise_lon_stddev', str(GNSS_LON_SDEV))
        gnss_bp.set_attribute('noise_lon_bias', str(GNSS_LON_BIAS))
        gnss_bp.set_attribute('sensor_tick', str(GNSS_PERIOD))
        relative_transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=0.0), carla.Rotation()) # TODO: Fix transform
        self.gnss = self.world.spawn_actor(gnss_bp, relative_transform, attach_to=self.ego)
        self.gnss.listen(self.gnss_cb)

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
        primary_imu_bp.set_attribute('sensor_tick', str(0.025))
        # primary_imu_bp.set_attribute('noise_accel_stddev_x', str(0.2))
        # primary_imu_bp.set_attribute('noise_accel_stddev_y', str(0.2))
        # primary_imu_bp.set_attribute('noise_accel_stddev_z', str(0.2))
        # primary_imu_bp.set_attribute('noise_gyro_stddev_x', str(0.03))
        # primary_imu_bp.set_attribute('noise_gyro_stddev_y', str(0.03))
        # primary_imu_bp.set_attribute('noise_gyro_stddev_z', str(0.03))
        primary_imu_bp.set_attribute('sensor_tick', str(0.025))
        # TODO: Add covariance. WSH.
        relative_transform = carla.Transform(carla.Location(x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.primary_imu = self.world.spawn_actor(primary_imu_bp, relative_transform, attach_to=self.ego)
        self.primary_imu.listen(self.primary_imu_cb)

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