'''
Package:   parade_controller
Filename:  controller.py
Author:    Daniel Vayman

Controller for the hoco parade that follows our flag
'''

from carla_msgs.msg import CarlaEgoVehicleControl
from geometry_msgs.msg import TransformStamped
from rosgraph_msgs.msg import Clock
from rclpy.node import Node

from sensor_msgs.msg import PointCloud2

from scipy.spatial.transform import Rotation as R

import rclpy
import ros2_numpy as rnp
import numpy as np
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class ParadeController(Node):

    def __init__(self):
        super().__init__('parade_controller_node')

        self.INTENSITY_THRESHOLD = 200
        self.DESIRED_DISTANCE = 3.7

        # PID control params
        self.Kp_Throttle = 1  # Proportional gain
        self.Kp_Steer = .3

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscriber
        self.lidar_left_sub = self.create_subscription(PointCloud2, 'velodyne_points', self.pointclouds_cb, 10)

        # TF listener

        # Publisher
        self.throttle_pub = self.create_publisher(
            CarlaEgoVehicleControl, '/carla/hero/vehicle_control_cmd', 10)
        
        # Debug publisher
        self.transformed_lidar_pub = self.create_publisher(PointCloud2, '/lidar_tfed', 1)

    def pointclouds_cb(self, msg: PointCloud2):

        pcd: np.array = rnp.numpify(msg)



        # Removes all points below the intensity threshold

        self.get_logger().info(f"Orig: {pcd.shape}")
        pcd = pcd[pcd['intensity'] > self.INTENSITY_THRESHOLD]
        self.get_logger().info(f"Now: {pcd.shape}")


        t = TransformStamped()

        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to {msg.header.frame_id}: {ex}')
            return

        t: TransformStamped
        q = t.transform.rotation

        r = R.from_quat([q.x, q.y, q.z, q.w])

        x = pcd['x']
        y = pcd['y']
        z = pcd['z']

        xyz = np.vstack((x, y, z)).T

        xyz = R.apply(r, xyz)

        pcd['x'] = xyz[:,0]
        pcd['y'] = xyz[:,1]
        pcd['z'] = xyz[:,2]

        pcd['x'] += t.transform.translation.x
        pcd['y'] += t.transform.translation.y
        pcd['z'] += t.transform.translation.z

        x = pcd['x']
        y = pcd['y']
        z = pcd['z']
        xy = np.vstack((x, y)).T

        # Finds the center of mass of banner points, resulting in a 1D numpy array (x, y) with the center coordinate of the banner
        banner = np.mean(xy[:, :2], axis=0)
        

        # Finds the distance to the banner (distance_x) and how far it is to the right or left (distance_y)
        distance_x = abs(banner[0])
        distance_y = banner[1]

        # Error
        error_x = distance_x - self.DESIRED_DISTANCE
        error_y = distance_y

        # Throttle & steering values TODO: integral & derivative terms
        throttle = self.Kp_Throttle * error_x
        steer = self.Kp_Steer * error_y

        # Set msg fields and publish throttle value
        throttle_msg = CarlaEgoVehicleControl()

        if throttle < 0.:
            throttle = 0.

        if steer > 1.:
            steer = 1.
        elif steer < -1.:
            steer = -1.
        
        throttle_msg.throttle = (1. if throttle > 1. else throttle)
        throttle_msg.steer = -1.*steer


        self.get_logger().info("Distance X: " + str(distance_x))
        self.get_logger().info("Distance Y: " + str(distance_y))
        self.get_logger().info("Thr: " + str(throttle))

        self.throttle_pub.publish(throttle_msg)

        total_length = pcd['x'].shape[0]

        msg_array = np.zeros(total_length, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)
        ])

        msg_array['x'] = pcd['x']
        msg_array['y'] = pcd['y']
        msg_array['z'] = pcd['z']
        msg_array['intensity'] = pcd['intensity']

        tfed_msg: PointCloud2 = rnp.msgify(PointCloud2, msg_array)

        tfed_msg.header = msg.header
        tfed_msg.header.frame_id = 'base_link'

        self.transformed_lidar_pub.publish(tfed_msg)




def main(args=None):
    rclpy.init(args=args)

    parade_controller_node = ParadeController()

    rclpy.spin(parade_controller_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    parade_controller_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
