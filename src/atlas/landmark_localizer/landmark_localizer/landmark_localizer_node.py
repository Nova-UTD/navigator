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

import pymap3d as pm

cunorrected_log = open("uncorrected.csv", 'w')
corrected_log = open("corrected.csv", 'w')

class LandmarkLocalizerNode(Node):

    def __init__(self):
        super().__init__('landmark_localizer')
        self.initPubSub()
        self.gnss = Odometry() #gps estimate
        self.gnss.pose.pose.position.x = 2.0
        self.gnss.pose.pose.position.y = 2.0
        self.gnss.pose.pose.position.z = 2.0
        self.load_landmarks()
        
        self.max_landmark_difference = 4.0 #only correct if a known landmark is less than x meters from the observed
        self.correction = np.array([0,0,0])
        self.target_correction = np.array([0,0,0])
        self.correction_move_speed = 0.1

        self.do_sign_localization = True
        
    def calc_correction(self):
        self.get_logger().info(f"{self.correction} {self.target_correction}")
        return self.correction + (self.target_correction-self.correction)*self.correction_move_speed

    def load_landmarks(self):
        #hard code list of landmarks (sorry)
        self.landmarks = []
        l = Landmark() #STOP SIGN BY PHASE 3 COMING FROM REC CENTER
        l.center_point.x = -299.0
        l.center_point.y = -139.0
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN BY PHASE 3 COMING FROM REC CENTER
        l.center_point.x = -319.2613163102851
        l.center_point.y = -603.9380237274966
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #FIRE HYDRANT PHASE 3 BY REC CENTER
        l.center_point.x = -151.69859272249326
        l.center_point.y = -586.7810955816708
        l.center_point.z = 1.2
        l.label = l.FIRE_HYDRANT
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN PHASE 3 PARKING LOT
        l.center_point.x = -305.44632001416176
        l.center_point.y = -513.228774044821
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        # l = Landmark() #STOP SIGN BY PHASE 3 AND PARKING GARAGE
        # l.center_point.x = -300.56205839743967
        # l.center_point.y = -429.0077070418239
        # l.center_point.z = 1.2
        # l.label = l.STOP_SIGN
        l = Landmark() #STOP SIGN ACROSS THE STREET DIAGONALLY BY PHASE 3 AND PARKING GARAGE
        l.center_point.x = -317.56205839743967
        l.center_point.y = -411.0077070418239
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN IN PARKING LOT BY PARKING GARAGE
        l.center_point.x = -298.3001872598016
        l.center_point.y = -323.6502444482444
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN PAST LAB BUILDINGS
        l.center_point.x = -298.8049204824074
        l.center_point.y = -263.4724736543279
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #FIRE HYDRANT BY SECOND STOP SIGN BY REC CENTER
        l.center_point.x = -64.81375677117087
        l.center_point.y = -607.4311219078864
        l.center_point.z = 1.2
        l.label = l.FIRE_HYDRANT
        self.landmarks.append(l) 
        l = Landmark() #SECOND STOP SIGN BY REC CENTER
        l.center_point.x = -30.562687470500386
        l.center_point.y = -596.2162151202285
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #FIRST STOP SIGN BY REC CENTER
        l.center_point.x = 36.97954323171624 
        l.center_point.y = -597.8959074840187
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #RIGHT STOP SIGN PARKING LOT 2 BEFORE HUGE INTERSECTION
        l.center_point.x = 403.2352641276224
        l.center_point.y = 37.42771778471791
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #LEFT STOP SIGN PARKING LOT 2 BEFORE HUGE INTERSECTION
        l.center_point.x = 415.1452761071515 
        l.center_point.y = 58.97503635384894
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN BY PARKING LOT BEFURE HUGE INTERSECTION
        l.center_point.x = 459.74975032906593
        l.center_point.y = -69.22017668406002
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #LEFT SIDE STOP SIGN BY PARKING LOT BEFORE HUGE INTERSECTION
        l.center_point.x = 474.0894052286024
        l.center_point.y = -47.12080623430883
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN BEFORE HUGE INTERSECTION
        l.center_point.x = 493.32613979938577 
        l.center_point.y = -104.52346107588416
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN BY LOT D (ATEC)
        l.center_point.x = 565.8509748029343 
        l.center_point.y = -197.0851284711794
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #FAR STOP SIGN BY ATEC, KINDA SKETCH BUT DOESN'T GET RECOGNIZED 
        l.center_point.x = 518.6783352418486
        l.center_point.y = -431.5969739967253
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #FIRST STOP SIGN BY JSOM
        l.center_point.x = 478.617544002802
        l.center_point.y = -587.8603550776359
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #SECOND STOP SIGN BY JSOM
        l.center_point.x = 408.66122526618415
        l.center_point.y = -577.5742447015119
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #JSOM FIRE HYDRANT
        l.center_point.x = 311.8256177404206 
        l.center_point.y = -574.4557014711646
        l.center_point.z = 1.2
        l.label = l.FIRE_HYDRANT
        self.landmarks.append(l) 
        l = Landmark() #TRAFFIC CIRCLE HYDRANT
        l.center_point.x = 234.30549651357802
        l.center_point.y = -545.9329233597714
        l.center_point.z = 1.2
        l.label = l.FIRE_HYDRANT
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN BY START POINT (ADMIN BUILDING)
        l.center_point.x = 158.43694499107576
        l.center_point.y = 124.92202085281235
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN ACROSS FROM EINSTEIN BROS PARKING GARAGE
        l.center_point.x = 43.84403459688677
        l.center_point.y = 139.89790224497207
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() #STOP SIGN END OF RES HALLS
        l.center_point.x = -61.765944858204236
        l.center_point.y = 125.6109221047434
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        l = Landmark() ##STOP SIGN by atec
        l.center_point.x = 539.0
        l.center_point.y = -415.5
        l.center_point.z = 1.2
        l.label = l.STOP_SIGN
        self.landmarks.append(l) 
        self.get_logger().info(f"Loaded {len(self.landmarks)} landmarks!")
        draws = []
        for k in self.landmarks:
            draws.append(k)
        print(draws)
        self.draw_landmarks(draws, ColorRGBA(
            r=0.0,
            g=1.0,
            b=0.0,
            a=1.0
        ),
        ColorRGBA(
            r=0.0,
            g=1.0,
            b=1.0,
            a=1.0
        ),
        'map')
        
    def draw_landmarks(self, landmarks, color_hydrant, color_stop, loc='map'):
        markers = MarkerArray()
        for i, l in enumerate(landmarks):
            l_marker = Marker()
            l_marker.header.frame_id = 'map'
            l_marker.id = i
            l_marker.ns = 'landmarks' + loc
            l_marker.frame_locked = True
            l_marker.type = l_marker.CUBE
            l_marker.scale.x = 1.0
            l_marker.scale.y = 1.0
            l_marker.scale.z = 1.0
            if l.label == l.FIRE_HYDRANT:
                l_marker.color = color_hydrant
            if l.label == l.STOP_SIGN:
                l_marker.color = color_stop
            l_marker.lifetime.sec = 3000

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
        self.inital_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/localization/initialpose', self.init_pose_cb, 10
        )

    #called when we recieve a new landmark
    #compares its position to the closest map landmark of that same type 
    #returns base_link offset of gnss
    def correct(self, landmark_msg_tf, label):
        map_landmark, distance_from_observed = self.closest_landmark(landmark_msg_tf, label)
        if map_landmark == None:
            self.get_logger().warn(f"No map landmark to compare!")
            return (None, False, 9999999)
        if distance_from_observed >= self.max_landmark_difference:
            self.get_logger().warn(f"Observed landmark too far ({distance_from_observed} m) from map landmark!")
            return (None, False, 9999999)
        #compute correction using base_link landmark
        trans = self.compute_trans(landmark_msg_tf, map_landmark)
        self.get_logger().info(f"offset: {trans}")
        return (trans, True, distance_from_observed)
        

    def tf_gnss(self, center, label=0):
        translate = np.zeros(3)
        
        rot = R.from_quat([self.gnss.pose.pose.orientation.x, self.gnss.pose.pose.orientation.y, self.gnss.pose.pose.orientation.z, self.gnss.pose.pose.orientation.w])
        final = rot.apply(center)
        translate[0] = final[0] + self.gnss.pose.pose.position.x
        translate[1] = final[1] + self.gnss.pose.pose.position.y
        translate[2] = final[2] + self.gnss.pose.pose.position.z
        res = Landmark()
        res.center_point.x = translate[0]
        res.center_point.y = translate[1]
        res.center_point.z = translate[2]
        res.label = label
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
    
    #publishes the gnss offset by trans. trans is in base_link
    def pub_correction(self):
        #rot_q = R.from_dcm(rot_mat).as_quat() #x,y,z,w
        t = PoseWithCovarianceStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        self.correction = self.calc_correction()
        self.get_logger().info(f"correction {self.correction}")

        tf_trans = self.tf_gnss(self.correction)
        self.get_logger().info(f"tf_correction {tf_trans.center_point.x} {tf_trans.center_point.y} {tf_trans.center_point.z}")
        t.pose.pose.position.x = tf_trans.center_point.x
        t.pose.pose.position.y = tf_trans.center_point.y
        t.pose.pose.position.z = self.gnss.pose.pose.position.z#tf_trans.center_point.z
        t.pose.pose.orientation = self.gnss.pose.pose.orientation
        
        self.write_lat_long(t.pose.pose.position.x,t.pose.pose.position.y,t.pose.pose.position.z, corrected_log)
        self.write_lat_long(self.gnss.pose.pose.position.x,self.gnss.pose.pose.position.y,self.gnss.pose.pose.position.z, cunorrected_log)

        self.corrected_pub.publish(t)
    def write_lat_long(self, x, y, z, log):
        lat0 = 32.989487
        lon0 = -96.750437
        alt0 = 196.0
        lat, lon, alt = pm.enu2geodetic(
            float(x), float(y), float(z), lat0, lon0, alt0)
        log.write(f"{lat},{lon}\n")

    #R(bl + trans) = map
    # => trans = R'*map - bl
    #not doing rotations anymore
    def compute_trans(self, tf_landmark, map_landmark):
        b_i = tf_landmark.center_point
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
    def closest_landmark(self, landmark_msg, label):
        candidates = self.landmarks
        self.get_logger().info(f"detected landmark with id {label} ({landmark_msg.center_point.x} {landmark_msg.center_point.y} {landmark_msg.center_point.z})")
        closest = None
        closest_dist = 9999999.9
        loc = landmark_msg.center_point
        for landmark in candidates:
            if landmark.label != label:
                continue #don't match fire hydrants with stop signs
            a = landmark.center_point
            sqr_d = (loc.x-a.x)**2 + (loc.y-a.y)**2 #+ (loc.z-a.z)**2
            if (sqr_d < closest_dist):
                closest_dist = sqr_d
                closest = landmark
        return (closest, math.sqrt(closest_dist))
        
    def gnss_cb(self, msg):
        self.gnss = msg
        self.pub_correction()

    #recieves LandmarkArray message, transforms landmarks form base_link to gnss, corrects
    def landmark_cb(self, msg):
        if not self.do_sign_localization:
            return
        if (self.gnss == None):
            self.get_logger().warn(f"Landmark recieved but no gnss!")
            return
        msg_tf = [self.tf_gnss(np.array([landmark.center_point.x, landmark.center_point.y, landmark.center_point.z]), landmark.label) for landmark in msg.landmarks]
        self.draw_landmarks(msg_tf, ColorRGBA(
            r=1.0,
            g=0.0,
            b=1.0,
            a=1.0
        ),
        ColorRGBA(
            r=1.0,
            g=1.0,
            b=1.0,
            a=1.0
        ),
        'perceived')
        trans = np.zeros(3)
        min_dist = 9999999
        
        
        for l, m in zip(msg_tf, msg.landmarks):
            t, has_correction, d = self.correct(l, m.label)
            if not has_correction:
                continue #no correction found
            if d > min_dist:
                continue
            min_dist = d 
            trans = t
        if min_dist >= 999999:
            return
        if t is None:
            return
        #publish closest of all landmarks
        self.target_correction = t
        self.pub_correction()
    
    def init_pose_cb(self, pose):
        us = np.array([pose.pose.pose.position.x, pose.pose.pose.position.y, pose.pose.pose.position.z])
        p = np.array([self.gnss.pose.pose.position.x, self.gnss.pose.pose.position.y, self.gnss.pose.pose.position.z])
        t = us - p
        self.get_logger().warn(f"Manual correction {t}!")
        self.target_correction = t
        self.correction = t
        self.pub_correction()

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
