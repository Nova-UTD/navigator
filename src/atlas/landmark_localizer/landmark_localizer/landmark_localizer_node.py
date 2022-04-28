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
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, TransformStamped
from voltron_msgs.msg import Obstacle3D, Landmark, LandmarkArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

import math
import numpy as np
from scipy.spatial.transform import Rotation as R
from itertools import combinations


class LandmarkLocalizerNode(Node):

    def __init__(self):
        super().__init__('landmark_localizer')
        self.get_logger().info("Hello, world!")
        self.initPubSub()
        self.gnss = Odometry() #gps estimate
        self.gnss.pose.pose.position.x = 2.0
        self.gnss.pose.pose.position.y = 2.0
        self.gnss.pose.pose.position.z = 2.0
        self.load_landmarks()
        
        self.max_landmark_difference = 5.0 #only correct if a known landmark is less than x meters from the observed
        self.last_correction = [0,0,0]

    def load_landmarks(self):
        #key is label, value is list of known landmarks
        self.landmarks = {}
        l = Landmark()
        l.center_point.x = 10.0
        l.center_point.y = 20.0
        l.center_point.z = 30.0
        self.landmarks[0] = [l]
        draws = []
        for k in self.landmarks:
            for lm in self.landmarks[k]:
                draws.append(lm)
        print(draws)
        self.draw_landmarks(draws, ColorRGBA(
            r=0.0,
            g=1.0,
            b=0.0,
            a=1.0
        ),
        'map')
        
    def draw_landmarks(self, landmarks, color, loc='map'):
        markers = MarkerArray()
        for i, l in enumerate(landmarks):
            l_marker = Marker()
            l_marker.header.frame_id = 'map'
            l_marker.id = i
            l_marker.ns = 'landmarks' + loc
            l_marker.frame_locked = True
            l_marker.type = l_marker.CUBE
            l_marker.scale.x = 10.0
            l_marker.scale.y = 10.0
            l_marker.scale.z = 10.0
            l_marker.color = color
            l_marker.lifetime.sec = 300

            l_marker.pose.position.x = l.center_point.x
            l_marker.pose.position.y = l.center_point.y
            l_marker.pose.position.z = l.center_point.z
            markers.markers.append(l_marker)
        if loc=='map':
            self.map_landmark_viz_pub.publish(markers)
            return
        else:
            self.perceived_landmark_viz_pub.publish(markers)
            return

    def initPubSub(self):
        self.gnss_sub = self.create_subscription(
            Odometry, '/sensors/gnss/odom', self.gnss_cb, 10)
        self.landmark_sub = self.create_subscription(
            LandmarkArray, '/landmarks', self.landmark_cb, 10)
        self.corrected_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/corrected_gnss', 10)
        self.perceived_landmark_viz_pub = self.create_publisher(
            MarkerArray, '/viz/perceived_landmarks', 10
        )
        self.map_landmark_viz_pub = self.create_publisher(
            MarkerArray, '/viz/map_landmarks', 10
        )

    #called when we recieve a new landmark
    #compares its position to the closest map landmark of that same type 
    def correct(self, landmark_msg):
        if (self.gnss == None):
            self.get_logger().warn(f"Landmark recieved but no gnss!")
            return
        landmark_msg_tf = self.tf_gnss(landmark_msg)
        self.draw_landmarks([landmark_msg_tf], ColorRGBA(
            r=1.0,
            g=0.0,
            b=1.0,
            a=1.0
        ),
        'perceived')
        map_landmark, distance_from_observed = self.closest_landmark(landmark_msg_tf)
        if map_landmark == None:
            self.get_logger().warn(f"No map landmark to compare!")
            return
        if distance_from_observed >= self.max_landmark_difference:
            self.get_logger().warn(f"Observed landmark too far ({distance_from_observed} m) from map landmark!")
            return
        #compute correction using base_link landmark
        trans = self.compute_trans(landmark_msg, map_landmark)
        self.get_logger().info(f"{trans}")
        #publish tf
        self.pub_correction(trans, self.gnss.pose.pose)

    def tf_gnss(self, landmark):
        center = np.array([landmark.center_point.x, landmark.center_point.y, landmark.center_point.z])
        center[0] += self.gnss.pose.pose.position.x
        center[1] += self.gnss.pose.pose.position.y
        center[2] += self.gnss.pose.pose.position.z
        rot = R.from_quat([self.gnss.pose.pose.orientation.x, self.gnss.pose.pose.orientation.y, self.gnss.pose.pose.orientation.z, self.gnss.pose.pose.orientation.w])
        final = rot.apply(center)
        res = Landmark()
        res.center_point.x = center[0]
        res.center_point.y = center[1]
        res.center_point.z = center[2]
        return res
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
            db = np.array([b_i.x-b_j.x,b_i.y-b_j.y,0])
            dm = np.array([m_i.x-m_j.x,m_i.y-m_j.y,0])
            #compute rotation matrix
            self.get_logger().info(f"db {db}")
            self.get_logger().info(f"dm {dm}")
            m = self.rotation_matrix_from_vectors(db, dm)
            sum_mat += m 
            self.get_logger().info(f"mat {m}")
            num_mat += 1
        #take the avereage
        return sum_mat/num_mat
    
    #should just construct a tf from the supplied parameters
    def pub_correction(self, trans, gnss_pose):
        #rot_q = R.from_dcm(rot_mat).as_quat() #x,y,z,w
        t = PoseWithCovarianceStamped()
        self.last_correction = trans
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.pose.pose.position.x = trans[0] + gnss_pose.position.x
        t.pose.pose.position.y = trans[1] + gnss_pose.position.y
        t.pose.pose.position.z = trans[2] + gnss_pose.position.z
        t.pose.pose.orientation = gnss_pose.orientation
    
        self.corrected_pub.publish(t)

    #R(bl + trans) = map
    # => trans = R'*map - bl
    #not doing rotations anymore
    def compute_trans(self, bl_landmark, map_landmark):
        b_i = bl_landmark.center_point
        m_i = map_landmark.center_point
        #should be map - b (hence the negative)
        return -np.array([b_i.x-m_i.x, b_i.y-m_i.y, b_i.z-m_i.z])

    
    def rotation_matrix_from_vectors(self, vec1, vec2):
        """ Find the rotation matrix that aligns vec1 to vec2
        :param vec1: A 3d "source" vector
        :param vec2: A 3d "destination" vector
        :return mat: A transform matrix (3x3) which when applied to vec1, aligns it with vec2.
        """
        if np.linalg.norm(vec1-vec2) < 0.01:
            return np.eye(3) #do not do the calculation if the vectosr are very close
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
        candidates = self.landmarks[landmark_msg.label]
        closest = None
        closest_dist = 9999999.9
        loc = landmark_msg.center_point
        for landmark in candidates:
            a = landmark.center_point
            sqr_d = (loc.x-a.x)**2 + (loc.y-a.y)**2 + (loc.z-a.z)**2
            if (sqr_d < closest_dist):
                closest_dist = sqr_d
                closest = landmark
        return (closest, math.sqrt(closest_dist))
        
    def gnss_cb(self, msg):
        self.gnss = msg
        self.pub_correction(self.last_correction, self.gnss.pose.pose)

    def landmark_cb(self, msg):
        for l in msg.landmarks:
            self.correct(l)

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
