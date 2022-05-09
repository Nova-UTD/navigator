#!/usr/bin/python


from rclpy.node import Node
import rclpy

from geometry_msgs.msg import PoseWithCovarianceStamped

log = open("pose_log.csv", 'w')

class LandmarkLocalizerNode(Node):

    def __init__(self):
        super().__init__('landmark_localizer')
        self.initPubSub()
        
    
    def initPubSub(self):
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/pcl_pose', self.pose_cb, 10
        )

   def pose_cb(self, msg):
       self.write_pose_cov(t.pose.pose.position.x,t.pose.pose.position.y,t.pose.pose.position.z, t.pose.covariance[0], log)

    def write_pose_cov(self, x, y, z, cov, log):
        log.write(f"{x},{y},{z},{cov}\n")

def main(args=None):
    rclpy.init(args=args)

    node = LandmarkLocalizerNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
