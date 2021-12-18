
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster, TransformStamped

class Estimator(Node):

    m_x0 = 0.0 # Starting offsets in meters. Reset after each GPS update.
    m_y0 = 0.0
    m_z0 = 0.0
    m_vx0 = 0.0
    m_vy0 = 0.0
    m_vz0 = 0.0

    def __init__(self):
        super().__init__('estimator')
        self.gps_sub = self.create_subscription(
            Odometry,
            '/gps/odometry',
            self.gps_cb,
            10)

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_cb,
            10)
    
        self.base_link_br = TransformBroadcaster(self)

        self.m_old_time = self.get_clock().now().to_msg()

    def gps_cb(self, msg: Odometry):
        self.get_logger().info("Got GPS")
        v = msg.twist.twist
        self.m_vx0 = v.linear.x
        self.m_vy0 = v.linear.y
        self.m_vz0 = v.linear.z
    
    def imu_cb(self, msg):

        old_time = self.m_old_time
        new_time = msg.header.stamp

        if self.m_old_time is None:
            dt = 0.0
        elif old_time.sec == new_time.sec:
            dt = (new_time.nanosec - old_time.nanosec)*1e-9
        else:
            dt = (new_time.nanosec - old_time.nanosec)*1e-9+(new_time.sec - old_time.sec)

        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Calculate linear displacement
        # d = z_0 + Δt*v_o + 0.5*a*Δt^2
        t.transform.translation.x = self.m_x0 + dt*self.m_vx0 + 0.5*msg.linear_acceleration.x*(dt**2)
        # print("{:.2f}+{:.2f}*{:.2f}+{:2f}".format(self.m_x0, dt, self.m_vx0, msg.linear_acceleration.x))
        self.m_x0 = t.transform.translation.x
        
        t.transform.translation.y = self.m_y0 + dt*self.m_vy0 + 0.5*msg.linear_acceleration.y*(dt**2)
        print("{:.2f}+{:.2f}*{:.2f}+{:2f}".format(self.m_y0, dt, self.m_vy0, msg.linear_acceleration.y))
        self.m_y0 = t.transform.translation.y

        t.transform.translation.z = self.m_z0 + dt*self.m_vz0 + 0.5*(msg.linear_acceleration.z-9.81)*(dt**2)
        self.m_z0 = t.transform.translation.z

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        # q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        # Send the transformation
        self.base_link_br.sendTransform(t)

        self.m_old_time = new_time
        


def main(args=None):
    rclpy.init(args=args)

    viz_subscriber = Estimator()

    rclpy.spin(viz_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    viz_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()