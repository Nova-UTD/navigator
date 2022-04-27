#!/usr/bin/python


from rclpy.node import Node
import rclpy

#tf2
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import tf2_py
import tf2_ros
from tf2_ros.buffer import Buffer
import tf2_msgs
from tf2_ros import TransformException, TransformStamped

#msgs
from nav_msgs.msg import Odometry  # For GPS
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped
from voltron_msgs.msg import Obstacle3D

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from itertools import combinations


class LandmarkLocalizerNode(Node):

    def __init__(self):
        super().__init__('landmark_localizer')
        self.get_logger().info("Hello, world!")
        self.load_landmarks()
        self.initPubSub()
        self.max_landmark_difference = 5.0 #only correct if a known landmark is less than x meters from the observed
        self.corners_to_use = [0,3,4,7] #bounding box corners (should be facing the car to be more reliable) (not if this is correct)
        

    def load_landmarks(self):
        self.landmarks = {}
        #thinking a dict of List[Obstacle3D]. we can assign an id for the sign (the key of the dict) and record the bounding box
        pass

    def initPubSub(self):
        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10)
        self.landmark_sub = self.create_subscription(
            Obstacle3D, 'xxx', self.landmark_cb, 10)
        self.corrected_pub = self.create_publisher(
            Pose, '/corrected_gnss', 10)

    #called when we recieve a new landmark
    #compares its position to the closest map landmark of that same type 
    def correct(self, landmark_msg):
        if (self.gnss == None):
            self.get_logger().warn(f"Landmark recieved but no gnss!")
            return
        #TODO: tf landmark_msg from base_link using GNSS
        landmark_msg_tf = tf_gnss(landmar_msg)
        map_landmark, distance_from_observed = self.closest_landmark(landmark_msg)
        if map_landmark == None:
            self.get_logger().warn(f"No map landmark to compare!")
            return
        if distance_from_observed >= self.max_landmark_difference:
            self.get_logger().warn(f"Observed landmark too far ({distance_from_observed} m) from map landmark!")
            return
        #compute correction using base_link landmark
        rot_mat = self.compute_rot(landmark_msg, map_landmark)
        trans = self.compute_trans(landmark_msg, map_landmark, rot_mat)
        #publish tf
        self.calc_pub_tf(rot_mat, trans)

        
    #computes the inverse rotation matrix by comparing the difference of pairs of points from each landmark
    #bl landmark is in base_link, map_landmark is in map
    #b = point on bl_landmark, m=point on map_landmark, R rotatation matrix
    #R*(b_i - b_j) = (m_i - m_j)
    def compute_rot(self, bl_landmark, map_landmark):
        sum_mat = np.zeros((3,3))
        num_mat = 0
        #im sure this could be made into a numpy 2 liner
        for i,j in combinations(self.corners_to_use, 2):
            #compute all pairs of offset vectors
            b_i = bl_landmark.bounding_box.corners[i]
            b_j = bl_landmark.bounding_box.corners[j]
            m_i = map_landmark.bounding_box.corners[i]
            m_j = map_landmark.bounding_box.corners[j]
            db = np.array([b_i[0]-b_j[0],b_i[1]-b_j[1],b_i[2]-b_j[2]])
            dm = np.array([m_i[0]-m_j[0],m_i[1]-m_j[1],m_i[2]-m_j[2]])
            #compute rotation matrix
            sum_mat += rotatation_matrix_from_vectors(db, dm)
            num_mat += 1
        #take the avereage
        return sum_mat/num_mat
    
    #should just construct a tf from the supplied parameters
    def calc_pub_tf(self, rot_mat, trans):
        rot_q = R.from_matrix(rot_mat).as_quat() #x,y,z,w
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()
   
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "base_link"
        t.child_frame_id = "landmark_tf"
        t.transform.translation.x = trans.x
        t.transform.translation.y = trans.y
        t.transform.translation.z = trans.z
        t.transform.rotation.x = rot_q[0]
        t.transform.rotation.y = rot_q[1]
        t.transform.rotation.z = rot_q[2]
        t.transform.rotation.w = rot_q[3]
    
        br.sendTransform(t)

    #R(bl + trans) = map
    # => trans = R'*map - bl
    def compute_trans(self, bl_landmark, map_landmark, rot_mat):
        inverse_rot = np.linalg.inv(rot_mat)
        sum_t = np.zeros(3)
        for i in self.corners_to_use:
            b_i = bl_landmark.bounding_box.corners[i]
            m_i = map_landmark.bounding_box.corners[i]
            bl = np.array([b_i[0], b_i[1], b_i[2]])
            mp = np.array([m_i[0], m_i[1], m_i[2]])
            sum_t += inverse_rot.dot(mp) - bl
        return sum_t/len(self.corners_to_use)

    
    def rotation_matrix_from_vectors(vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        a, b = (vec1 / np.linalg.norm(vec1)).reshape(3), (vec2 / np.linalg.norm(vec2)).reshape(3)
        v = np.cross(a, b)
        c = np.dot(a, b)
        s = np.linalg.norm(v)
        kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
        return rotation_matrix
    #finds closest landmark of the same type based off center
    #landmark_msg should be already transformed by GNSS when passed here
    #returns (Obstacle3D, float: the distance from the landmark_msg)
    def closest_landmark(self, landmark_msg):
        candidates = self.landmarks[landmark_msg.id]
        closest = None
        closest_dist = 9999999.9
        loc = landmark_msg.bounding_box.center.position
        for landmark in candidates:
            a = landmark.bounding_box.center.position
            sqr_d = (loc.x-a.x)**2 + (loc.y-a.y)**2 + (loc.z-a.z)**2
            if (sqr_d < closest_dist):
                closest_dist = sqr_d
                closest = landmark
        return (closest, math.sqrt(closest_dist))
        
    def gnss_cb(self, msg):
        self.gnss = msg
    def landmark_cb(self, msg):
        self.correct(msg)

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
