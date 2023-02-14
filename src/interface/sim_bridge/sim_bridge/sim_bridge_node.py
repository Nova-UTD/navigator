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
✓ CARLA ground truths for
    ✓ Detected objects
    ✓ Car's odometry (position, orientation, speed)
    ✓ CARLA virtual bird's-eye camera (/carla/birds_eye_rgb)

Todos:
- Specific todos are dispersed in this script. General ones are here.
- Ensure all sensors publish in ROS coordinate system, NOT Unreal Engine's.

'''

from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Point, Quaternion, Vector3, PoseWithCovariance, TwistWithCovarianceStamped
from nova_msgs.msg import PeddlePosition, SteeringPosition, Obstacle3DArray, Obstacle3D, BoundingBox3D
from nav_msgs.msg import Odometry  # For GPS, ground truth
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image  # For cameras
from std_msgs.msg import Bool, Header, Float32
from os.path import exists
import math
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py
from tf2_ros.buffer import Buffer
import tf2_msgs
from tf2_ros import TransformException, TransformStamped
import pymap3d
from cv_bridge import CvBridge
import numpy as np
import ros2_numpy as rnp
from rclpy.node import Node
import rclpy
import sys
sys.path.append('/home/share/carla/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg')
import carla
import random
import sim_bridge.scenarios as sc

# #SCENARIO TO RUN
# SCENARIO = sc.car_in_junction

# GLOBAL CONSTANTS
# TODO: Move to ROS param file, read on init. WSH.
CLIENT_PORT = 2000

GNSS_PERIOD = 1/(2.0)  # 2 Hz
GROUND_TRUTH_OBJ_PERIOD = 1/(2.0)  # 2 Hz (purposely bad)
GROUND_TRUTH_ODOM_PERIOD = 1/(10.0)  # 10 Hz
LIDAR_PERIOD = 1/(10.0)  # 10 Hz
SEMANTIC_LIDAR_PERIOD = 1/(2.0)  # 10 Hz
SPEEDOMETER_PERIOD = 1/(10.0)  # 10 Hz
STEERING_ANGLE_PERIOD = 1/(10.0)  # 10 Hz
OBSTACLE_QTY_VEHICLE = 2  # Spawn n cars
OBSTACLE_QTY_PED = 0  # Spawn n peds

# Map-specific constants
MAP_ORIGIN_LAT = 0.0  # degrees
MAP_ORIGIN_LON = 0.0  # degrees

# Sensor noise constants
M_TO_DEG = 9e-6  # APPROXIMATE! WSH.

# Degrees -  https://carla.readthedocs.io/en/latest/ref_sensors/#gnss-sensor
GNSS_ALT_BIAS = random.uniform(0.25, 1.0)*M_TO_DEG*0.3
GNSS_ALT_SDEV = 0.5*M_TO_DEG*0.3
GNSS_LAT_BIAS = random.uniform(0.25, 1.0)*M_TO_DEG*0.3
GNSS_LAT_SDEV = 0.5*M_TO_DEG*0.3
GNSS_LON_BIAS = random.uniform(0.25, 1.0)*M_TO_DEG*0.3
GNSS_LON_SDEV = 0.5*M_TO_DEG*0.3

# Publish a true map->base_link transform. Disable this if
# another localization algorithm (ukf, ndt, etc.) is running! WSH.
PULBISH_MAP_BL_TRANSFORM = False

sys.path.append(
    '/home/share/carla/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg')

sys.path.append(
    '/home/share/carla/PythonAPI/carla/dist/carla-0.9.12-py3.7-linux-x86_64.egg')


# For lidar data manipulation/conversion. WSH.

# For camera data conversion. WSH.

# For lat/lon-> ENU conversions. WSH.


# Message definitons


class SimBridgeNode(Node):

    # Straight-up copied from carla_ros_bridge. WSH.
    def get_color_image(self, carla_image: carla.Image, encoding: str = 'bgra8'):
        """
        Function to transform the received carla camera data into a ROS image message
        """
        carla_image_data_array = np.ndarray(
            shape=(carla_image.height, carla_image.width, 4),
            dtype=np.uint8, buffer=carla_image.raw_data)
        img_msg = self.cv_bridge.cv2_to_imgmsg(
            carla_image_data_array, encoding=encoding)
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = '/base_link'

        return img_msg

    # Straight-up copied from carla_ros_bridge. WSH.
    def get_depth_image(self, carla_image: carla.Image, encoding: str = 'passthrough'):
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
            frame_id='lidar_right'
        )

        lidar_data = np.fromstring(
            bytes(data.raw_data), dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 4), 4))
        lidar_data.dtype = [
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

    def gnss_cb(self, data: carla.GnssMeasurement):
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
        # This should be 0.0. We don't care about altitude. WSH.
        posewithcov.pose.position.z = enu_coords[2]

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

        accuracy = 1.0  # Meters, s.t. pos.x = n +/- accuracy

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
        speed = math.floor(
            math.sqrt((vel.x ** 2) + (vel.y ** 2) + (vel.z ** 2))/0.447) * 0.447
        if (speed > 10.2):
            cmd.throttle = 0.0  # Cap our speed at 23 mph. WSH.
            # self.get_logger().warn("Your speed of {} has maxed out".format(speed))
        # self.ego.apply_control(cmd)

    def sem_lidar_cb(self, data: carla.SemanticLidarMeasurement):

        rand_pick = random.uniform(0.0, 1.0)
        if (rand_pick > 0.02):
            return

        # For output classes, see https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
        # Roads: yellow
        # Other ground: green
        # Vehicles (incl. bikes): purple
        # Signs: blue
        # Other: off-white
        road_color_rgb = [252, 255, 166]  # FCFFA6, yellow
        ground_color_rgb = [193, 255, 215]  # C1FFD7, light green
        vehicle_color_rgb = [202, 184, 255]  # CAB8FF, purple
        sign_color_rgb = [181, 222, 255]  # B5DEFF, light blue
        other_color_rgb = [249, 248, 248]  # F9F8F8, off-white

        # for det in data:
        #     self.get_logger().info("{}".format(det))

        # Taken from carla_ros_bridge's "lidar.py". WSH.
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id='lidar_right'
        )

        lidar_data = np.fromstring(
            bytes(data.raw_data), dtype=np.float32)
        lidar_data = np.reshape(
            lidar_data, (int(lidar_data.shape[0] / 6), 6))
        lidar_data.dtype = [
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

    def add_obstacle(self, actor, obstacles, is_car):
        if actor.id == self.ego.id:
            return #motivational quote: we should not be an obstacle to ourselves

        ego_position = Point()
        ego_tf: carla.Transform = self.ego.get_transform()
        ego_position.x = ego_tf.location.x
        ego_position.y = ego_tf.location.y * -1
        ego_position.z = 0.0  # Force to zero

        obst = Obstacle3D()
        obst.id = actor.id
        obst.label = obst.CAR if is_car else obst.PEDESTRIAN
        obst.confidence = random.uniform(0.5, 1.0)

        bbox = actor.bounding_box

        # Set velocity
        actor_vel = actor.get_velocity()
        obst.velocity.x = actor_vel.x
        obst.velocity.y = actor_vel.y * -1  # Fix coordinate system
        obst.velocity.z = actor_vel.z

        actor_tf: carla.Transform = actor.get_transform()

        # add world space corner points
        corner_points = bbox.get_world_vertices(actor_tf)
        carla_to_our_ordering = [(0,0),(4,1),(6,2),(2,3)] #carla orders vertices in the box differently than we do. 
        for c, us in carla_to_our_ordering:
            tf_pt = corner_points[c]
            corner = Point()
            #change all coordinates to be relative to car ('base_link') instead of world ('map')
            corner.x = tf_pt.x - ego_position.x
            corner.y = -tf_pt.y - ego_position.y # change coordinate system
            corner.z = tf_pt.z - ego_position.z
            obst.bounding_box.corners[us] = corner
        
        obstacles.append(obst)

    def publish_true_boxes(self):

        vehicles = self.world.get_actors().filter('vehicle.*')
        pedestrians = self.world.get_actors().filter('walker.*')
        # self.get_logger().info("Publishing {} boxes.".format(len(vehicles)))

        obstacles = []
        for vehicle in vehicles:
            self.add_obstacle(vehicle, obstacles, True)
        for ped in pedestrians:
            self.add_obstacle(ped, obstacles, False)
        obstacles_msg = Obstacle3DArray()
        obstacles_msg.obstacles = obstacles
        obstacles_msg.header.stamp = self.get_clock().now().to_msg()
        obstacles_msg.header.frame_id = 'map'

        self.ground_truth_obst_pub.publish(obstacles_msg)
        # self.get_logger().info("{}".format(obstacles))

    def pub_speedometer(self):
        vel = self.ego.get_velocity()
        twist_linear = TwistWithCovarianceStamped()
        speed = math.floor(
            math.sqrt((vel.x ** 2) + (vel.y ** 2) + (vel.z ** 2))/0.447) * 0.447
        twist_linear.twist.twist.linear.x = speed
        # Our speedometer is accurate to ~0.5 m/s. Covariance is the square of stdev.
        twist_linear.twist.covariance[0] = 0.25
        twist_linear.header.frame_id = 'base_link'
        twist_linear.header.stamp = self.get_clock().now().to_msg()

        self.speedometer_pub.publish(twist_linear)

    def pub_steering_angle(self):
        front_left_wheel = self.ego.get_wheel_steer_angle(
            carla.VehicleWheelLocation.FL_Wheel)
        front_right_wheel = self.ego.get_wheel_steer_angle(
            carla.VehicleWheelLocation.FR_Wheel)
        tricycle_angle = (front_left_wheel + front_right_wheel)/2
        # Convert to right-handed, degrees to radians
        tricycle_angle *= -math.pi/180
        msg = SteeringPosition()
        msg.data = tricycle_angle
        self.steering_angle_pub.publish(msg)

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
        ego_position.z = 0.0  # Force to zero

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

        ego_vel = self.ego.get_velocity()
        twist_linear = Vector3()
        twist_linear.x = ego_vel.x
        twist_linear.y = ego_vel.y * -1
        twist_linear.z = ego_vel.z
        odom.twist.twist.linear = twist_linear

        self.ground_truth_odom_pub.publish(odom)

        # Publish tf if enabled
        if PULBISH_MAP_BL_TRANSFORM:
            self.last_tf = TransformStamped()
            self.last_tf.header = odom.header
            self.last_tf.child_frame_id = odom.child_frame_id
            translation = Vector3(
                x=odom.pose.pose.position.x,
                y=odom.pose.pose.position.y,
                z=odom.pose.pose.position.z
            )
            self.last_tf.transform.translation = translation
            self.last_tf.transform.rotation = Quaternion(
                x=ego_quat[0],
                y=ego_quat[1],
                z=ego_quat[2],
                w=ego_quat[3]
            )
            self.tf_broadcaster.sendTransform(self.last_tf)

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
            1
        )

        self.front_depth_pub = self.create_publisher(
            Image,
            '/camera_front/depth',
            1
        )

        self.front_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar_right/points_raw',
            1
        )

        self.front_rgb_pub = self.create_publisher(
            Image,
            '/camera_front/rgb',
            1
        )

        self.rear_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar_left/points_raw',
            1
        )

        self.gnss_pub = self.create_publisher(
            Odometry,
            '/gnss/odom',
            1
        )

        self.ground_truth_odom_pub = self.create_publisher(
            Odometry,
            '/carla/odom',
            1
        )

        self.ground_truth_obst_pub = self.create_publisher(
            Obstacle3DArray,
            '/objects',
            1
        )

        self.primary_imu_pub = self.create_publisher(
            Imu,
            '/imu_primary/data',
            1
        )

        self.speedometer_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/can/speedometer_twist',
            1
        )

        self.steering_angle_pub = self.create_publisher(
            SteeringPosition,
            '/can/steering_angle',
            10
        )

        self.sem_lidar_pub = self.create_publisher(
            PointCloud2,
            '/lidar/semantic',
            1
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

        self.steering_angle_timer = self.create_timer(
            STEERING_ANGLE_PERIOD, self.pub_steering_angle
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.connect_to_carla()

    def connect_to_carla(self):
        # Connect to client, load world
        self.get_logger().info("Connecting to CARLA on port {}".format(CLIENT_PORT))
        self.client = carla.Client('localhost', CLIENT_PORT)
        self.client.set_timeout(20.0)
        
        scenario_manager = sc.ScenarioManager(self)
        scenario_manager.normal(carla_autopilot=True, num_cars = 50, num_ped = 30)

        self.get_logger().info("Started scenario!")
        

    def add_ego_sensors(self):

        front_lidar_tf = carla.Transform(
            carla.Location(x=3.4, y=0.902, z=0.7876),
            # DEGREES, left-handed. Come on, CARLA...
            carla.Rotation(roll=0.85943669, pitch=1.71887339, yaw=83.0788803)
        )

        # Attach Lidar sensor
        lidar_bp = self.blueprint_library.find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', '16')  # VLP-16
        lidar_bp.set_attribute('sensor_tick', str(LIDAR_PERIOD))
        lidar_bp.set_attribute('rotation_frequency', '40')
        # lidar_bp.set_attribute('points_per_second', '11,200')
        relative_transform = front_lidar_tf
        self.front_lidar = self.world.spawn_actor(
            lidar_bp, relative_transform, attach_to=self.ego)
        self.front_lidar.listen(self.front_lidar_cb)

        # Attach GNSS sensor
        gnss_bp = self.blueprint_library.find('sensor.other.gnss')
        # We can add noise here.
        gnss_bp.set_attribute('noise_alt_stddev', str(GNSS_ALT_SDEV))
        gnss_bp.set_attribute('noise_alt_bias', str(GNSS_ALT_BIAS))
        gnss_bp.set_attribute('noise_lat_stddev', str(GNSS_LAT_SDEV))
        gnss_bp.set_attribute('noise_lat_bias', str(GNSS_LAT_BIAS))
        gnss_bp.set_attribute('noise_lon_stddev', str(GNSS_LON_SDEV))
        gnss_bp.set_attribute('noise_lon_bias', str(GNSS_LON_BIAS))
        gnss_bp.set_attribute('sensor_tick', str(GNSS_PERIOD))
        relative_transform = carla.Transform(carla.Location(
            x=0.0, y=0.0, z=0.0), carla.Rotation())  # TODO: Fix transform
        self.gnss = self.world.spawn_actor(
            gnss_bp, relative_transform, attach_to=self.ego)
        self.gnss.listen(self.gnss_cb)

        # Attach bird's-eye camera
        birds_eye_cam_bp = self.blueprint_library.find('sensor.camera.rgb')
        birds_eye_cam_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(
            x=0.0, y=0.0, z=20.0), carla.Rotation(pitch=-90.0))
        self.birds_eye_cam = self.world.spawn_actor(
            birds_eye_cam_bp, relative_transform, attach_to=self.ego)
        self.birds_eye_cam.listen(self.birds_eye_cam_cb)

        # Attach front rgb camera
        front_rgb_bp = self.blueprint_library.find('sensor.camera.rgb')
        front_rgb_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(
            x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.front_rgb = self.world.spawn_actor(
            front_rgb_bp, relative_transform, attach_to=self.ego)
        self.front_rgb.listen(self.front_rgb_cb)

        # Attach front depth camera
        front_depth_bp = self.blueprint_library.find('sensor.camera.depth')
        front_depth_bp.set_attribute('sensor_tick', str(0.1))
        relative_transform = carla.Transform(carla.Location(
            x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.front_depth = self.world.spawn_actor(
            front_depth_bp, relative_transform, attach_to=self.ego)
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
        relative_transform = carla.Transform(carla.Location(
            x=2.0, y=0.0, z=2.0), carla.Rotation(pitch=0.0))
        self.primary_imu = self.world.spawn_actor(
            primary_imu_bp, relative_transform, attach_to=self.ego)
        self.primary_imu.listen(self.primary_imu_cb)

    def get_random_spawn(self) -> carla.Transform:
        spawn_points = self.world.get_map().get_spawn_points()
        return random.choice(spawn_points)


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