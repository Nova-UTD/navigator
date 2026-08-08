"""
Package:   frame_tf_service
Filename:  frame_tf_service.py
Authors:   David Homiller, Saishravan Muthukrishnan, Yusuf Shaikh (AI was used for help with some debugging, math, learning information about ROS2, and packages.xml and CmakeLists.txt additions)
Email:     david.homiller@utdallas.edu, saishravan.muthukrishnan@utdallas.edu, yusuf.shaikh@utdallas.edu
Copyright: 2021, Nova UTD
License:   MIT License

TODO: Description
"""

# CV2 import for testing. Remove later.
import cv2
from cv_bridge import CvBridge, CvBridgeError

# Imports
import numpy as np
from math import hypot
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from scipy.spatial.transform import Rotation as R
import image_geometry
from rclpy.serialization import serialize_message, deserialize_message


# Message Imports
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import TransformStamped, Vector3, Point
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
from frame_tf_serv.srv import FrameTF

MAX_PIXEL_RADIUS = 5.0


class FrameTFService(Node):
    
    def __init__(self):
        super().__init__('FrameTFService')
        
        self.declare_parameter('seg_topic', '/semantics/semantic0')
        self.srv = self.create_service(
            FrameTF,
            'frame_tf',
            self.callback
        )
        self.serialized_message = None
        self.timer = self.create_timer(0.02, self.timer_cb) # should i use a timer? what should the period be?
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.ex = None

        # Declared for publishing msgs 
        self.stamp = Time() 

        # Subcribes to raw lidar data
        self.lidar_sub = self.create_subscription(
            PointCloud2, '/lidar', self.lidar_callback, 10)
        
        # holds LIDAR transform stamp
        self.t = None

        # Subscribes to camera info
        # NOTE: this CARLA setup publishes rgb_front, not rgb_center (center has 0 publishers).
        self.create_subscription(
            CameraInfo, '/carla/hero/rgb_front/camera_info', self.rgb_front_camera_info_cb, 10)
        self.create_subscription(
            CameraInfo, '/carla/hero/rgb_center/camera_info', self.rgb_center_camera_info_cb, 10)
        self.create_subscription(
            CameraInfo, '/carla/hero/rgb_left/camera_info', self.rgb_left_camera_info_cb, 10)
        self.create_subscription(
            CameraInfo, '/carla/hero/rgb_right/camera_info', self.rgb_right_camera_info_cb, 10)
        self.create_subscription(
            CameraInfo, '/carla/hero/rgb_back/camera_info', self.rgb_back_camera_info_cb, 10)

        
        # Subscribes to clock
        self.clock_sub = self.create_subscription(
            Clock, '/clock', self.clock_cb, 10)
        
        self.cam_arr = None
        self.latest_cloud: Optional[PointCloud2] = None
        self.lid_arr: Optional[np.ndarray] = None
        
        # stores CameraInfo per camera (raw msg — has .k, .width, .height, .header.frame_id)
        self.rgb_front_cam_model: Optional[CameraInfo] = None
        self.rgb_center_cam_model: Optional[CameraInfo] = None
        self.rgb_left_cam_model: Optional[CameraInfo] = None
        self.rgb_right_cam_model: Optional[CameraInfo] = None
        self.rgb_back_cam_model: Optional[CameraInfo] = None

    def timer_cb(self):
            pass

    def get_camera_info(self, camera_name: str) -> Optional[CameraInfo]:
        """Look up cached CameraInfo by camera label."""
        cameras = {
            "rgb_front": self.rgb_front_cam_model,
            "rgb_center": self.rgb_center_cam_model,
            "rgb_left": self.rgb_left_cam_model,
            "rgb_right": self.rgb_right_cam_model,
            "rgb_back": self.rgb_back_cam_model,
        }
        return cameras.get(camera_name)

    def rgb_front_camera_info_cb(self, msg: CameraInfo):
        self.rgb_front_cam_model = msg
        
    def rgb_center_camera_info_cb(self, msg: CameraInfo):
        self.rgb_center_cam_model = msg
        
    
    def rgb_right_camera_info_cb(self, msg: CameraInfo):
        self.rgb_right_cam_model = msg

    def rgb_left_camera_info_cb(self, msg: CameraInfo):
        self.rgb_left_cam_model = msg
    
    def rgb_back_camera_info_cb(self, msg: CameraInfo):
        self.rgb_back_cam_model = msg    
    
    def clock_cb(self, msg):
        """!
        Updates the clock for message headers.
        @param msg[Clock]   The clock message.
        """

        self.stamp.sec = msg.clock.sec
        self.stamp.nanosec = msg.clock.nanosec
        
    def lidar_callback(self, lidar_msg: PointCloud2):
        """ stores LIDAR transform information
        
        Args:
            lidar_msg (PointCloud2)
            
        Returns:
            None
        """
        
        # Get LIDAR points into np array
        self.lid_arr = np.frombuffer(lidar_msg.data, dtype=np.float32)
        self.lid_arr = np.reshape(self.lid_arr, (-1, 4))
        self.lid_arr = self.lid_arr[:,0:3]

        # store point cloud for pixel_to_world
        self.latest_cloud = lidar_msg


    def attach_depth(self, camera):
        """ Applies camera intrinsics/extrinsics to LIDAR point cloud and stores it
        
        Args:
            camera (string)
        Returns:
            None
        """
        t = TransformStamped()
        try:
            t = self.tf_buffer.lookup_transform(
                'hero/'+camera, 'base_link', rclpy.time.Time(),timeout=rclpy.duration.Duration(seconds=2.0))
            #print("found transform!")
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform to camera frameawefew: {ex}')
            return

        # Rotate to camera frame
        q = t.transform.rotation
        tf_rotation: R = R.from_quat([q.x, q.y, q.z, q.w])
        cam_arr = tf_rotation.apply(self.lid_arr)

        # Translate to camera frame
        cam_arr += [t.transform.translation.x,
                             t.transform.translation.y,
                             t.transform.translation.z]

        # Only keep points in front of the camera
        cam_arr = cam_arr[cam_arr[:, 2] > 0]              
        self.cam_arr = cam_arr
    
    def camImage_to_world(self,camera,stamp):
        """ Converts all pixels in a camera image into world coordinates and stores in a blob
        
        Args:
            camera (string)
            stamp (Time)
        Returns:
            None
        """
        while True:
            try:
                # get rotation matrix
                r_wc = self.tf_buffer.lookup_transform('hero/'+camera, 'base_link', stamp,timeout=rclpy.duration.Duration(seconds=2.0)).transform.rotation
                R_wc = R.from_quat([r_wc.x, r_wc.y, r_wc.z, r_wc.w]).as_matrix()
                
                # get translation matrix
                t_wc = self.tf_buffer.lookup_transform('hero/'+camera, 'base_link', stamp,timeout=rclpy.duration.Duration(seconds=2.0)).transform.translation
                t_wc = np.array([t_wc.x,t_wc.y,t_wc.z])
                
                # get tranformed LIDAR coords                
                self.attach_depth(camera)
                
                #  Apply camera rotation and translation to transformed LIDAR points
                x_w = R_wc.T@self.cam_arr.T + t_wc.reshape(3,1)
                
                # store as blob
                self.serialized_message = x_w.tobytes()
                break;
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform to camera frameawefew: {ex}')
                continue

    def world_to_pixel(self, camera: str, stamp: Time, world_coord) -> bool:
        """
        Project a map-frame 3D point onto the camera image.

        Process:
          1. Load CameraInfo for `camera` (intrinsics + optical frame id).
          2. Look up TF that expresses map points in the camera frame
             (stamp 0/0 → latest TF; otherwise use the request stamp).
          3. Apply the rigid transform: p_cam = R · P_map + t.
          4. Reject non-finite points or points behind the lens (z_c <= 0).
          5. Pinhole-project with K: u = fx·x_c/z_c + cx, v = fy·y_c/z_c + cy.
          6. Store (u, v) as float64 bytes in self.serialized_message.

        Returns:
            True on success (message filled), False if CameraInfo/TF/geometry fails.
        """
        # Get intrinsics (fx, fy, cx, cy) and the camera's TF frame name
        camera_info = self.get_camera_info(camera)
        if camera_info is None:
            self.get_logger().info(f"No CameraInfo yet for '{camera}'")
            return False

        # p_map = the 3D point in the world/map frame (what the caller asked about)
        p_map = np.asarray(world_coord, dtype=np.float64)
        camera_frame = camera_info.header.frame_id

        # if client sends time 0/0, use the latest pose
        tf_time = (
            rclpy.time.Time()
            if (stamp.sec == 0 and stamp.nanosec == 0)
            else stamp
        )

        # express map points in this camera's frame
        try:
            tf = self.tf_buffer.lookup_transform(
                camera_frame, "map", tf_time,
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as ex:
            self.get_logger().info(f"world_to_pixel TF failed: {ex}")
            return False

        # Rigid transform: rotate the point, then slide it (R · p_map + t)
        # Result p_cam is the same physical point, but measured from the camera
        q = tf.transform.rotation
        t = tf.transform.translation
        Rm = R.from_quat([q.x, q.y, q.z, q.w])
        p_cam = Rm.apply(p_map) + np.array([t.x, t.y, t.z])

        # In camera coords, +Z points out through the lens. reject points behind camera
        x_c, y_c, z_c = p_cam
        if not np.all(np.isfinite(p_cam)) or z_c <= 0:
            self.get_logger().info(
                f"world_to_pixel: point not in front of camera (z_c={z_c})")
            return False

        # Pinhole projection: divide by depth, then scale/shift into pixel coords
        fx, fy, cx, cy = self.get_intrinsics(camera_info)
        u = fx * (x_c / z_c) + cx
        v = fy * (y_c / z_c) + cy

        # Save (u, v) as float64 bytes so the client can unpack them
        self.serialized_message = np.asarray([u, v], dtype=np.float64).tobytes()
        self.get_logger().info(f"world_to_pixel → (u={u:.2f}, v={v:.2f})")
        return True

    def get_intrinsics(self, camera_info: CameraInfo) -> Tuple[float, float, float, float]:
        """
        Get focal length and principal point from the camera info matrix.
        Returns (fx, fy, cx, cy).
        """
        k = camera_info.k
        fx, fy = k[0], k[4]
        cx, cy = k[2], k[5]
        return fx, fy, cx, cy
    
    def cloud_to_xyz(self, cloud: PointCloud2) -> np.ndarray:
        # extracts the X, Y, and Z 3D spatial coordinates from a ROS 2 PointCloud2 message and loads them into a numpy array
        return pc2.read_points_numpy(cloud, field_names=('x','y','z'))
    
    def transform_cloud(
        self,
        pts: np.ndarray,
        target_frame: str,
        source_frame: str,
        stamp: Time,
    ) -> np.ndarray:
        """
        pts: (N,3) in source_frame  ->  (N,3) in target_frame.
        returns the transform that takes a point from source into target; applied as R · p + t
        """
        # get the transform that converts a point from source into target at time stamp
        tf = self.tf_buffer.lookup_transform(
            target_frame, source_frame, stamp,
            timeout=rclpy.duration.Duration(seconds=0.2))   # NOT while True
        q = tf.transform.rotation # quaternion (orientation of source relative to target)
        t = tf.transform.translation # 3-vector (where the source origin sits in the target frame)
        Rm = R.from_quat([q.x, q.y, q.z, q.w]) # convert quaternion into managed scipy Rotation object
        return Rm.apply(pts) + np.array([t.x, t.y, t.z]) # apply the rotation to every point in pts, then add the translation

    def pixel_to_world(
        self,
        camera_info: CameraInfo,
        image_stamp: Time,
        cloud: PointCloud2,
        u: float,
        v: float,
    ) -> Optional[np.ndarray]:
        """
        Associate an image pixel with a nearby LiDAR return and return its map XYZ.

        A pixel alone is underdetermined (a ray). This does nearby-ray association:
        project the latest cloud into the image and pick the nearest return within
        MAX_PIXEL_RADIUS (not an exact ray–surface intersection).

        Process:
          1. Read intrinsics (fx, fy, cx, cy) and camera / cloud frame ids.
          2. Parse the PointCloud2 into an (N,3) XYZ array.
          3. TF the cloud into the camera frame (for projection) and into map
             (for the answer). Prefer image_stamp for TF; else cloud stamp.
          4. For each return: drop non-finite / behind-camera (z_c <= 0) /
             off-image projections; project with the pinhole model.
          5. Among returns within MAX_PIXEL_RADIUS of (u, v), keep the nearest
             in image space; break near-ties with smaller z_c (frontmost).
          6. Return that return's map XYZ, or None if nothing is close enough.

        Returns:
            np.ndarray shape (3,) in map, or None on failure / no association.
        """
        fx, fy, cx, cy = self.get_intrinsics(camera_info)
        camera_frame = camera_info.header.frame_id  # where the camera lives in TF
        src = cloud.header.frame_id  # where the LiDAR points currently live

        points = self.cloud_to_xyz(cloud)  # (N,3) in the cloud's own frame
        if points.size == 0:
            return None

        # Prefer image stamp for TF when provided; fall back to cloud stamp
        stamp = image_stamp if (image_stamp.sec != 0 or image_stamp.nanosec != 0) else cloud.header.stamp

        # Same laser hit in two frames: project in camera, answer in map
        cloud_cam = self.transform_cloud(points, camera_frame, src, stamp)
        cloud_map = self.transform_cloud(points, "map", src, stamp)

        best = None  # (d_pixels, z_c, p_map)
        for p_cam, p_map in zip(cloud_cam, cloud_map):  # each is [x, y, z]
            x_c, y_c, z_c = p_cam
            if not np.all(np.isfinite(p_cam)) or z_c <= 0:  # skip bad / behind-camera
                continue

            # Project with pinhole formula
            ui = fx * (x_c / z_c) + cx
            vi = fy * (y_c / z_c) + cy
            if not (0 <= ui < camera_info.width and 0 <= vi < camera_info.height):
                continue

            # Image distance from this projected hit to the query pixel
            d = hypot(ui - u, vi - v)

            if d <= MAX_PIXEL_RADIUS:
                # Nearest pixel first; break near-ties by frontmost depth
                if best is None or (round(d, 1), z_c) < (round(best[0], 1), best[1]):
                    best = (d, z_c, p_map)

        if best is None:
            return None  # no LiDAR near this pixel
        return best[2]  # map point measured at cloud time
        
                
    
    def callback(self,request, response):
        """ Calls selected transformation function based on recieved message parameters, and returns serialized output
        
        Args:
            camera (string)
            stamp (Time)
            world_coord (3 Tuple)
        Returns:
            None
        """
        response.tf_success = False
        response.coords = bytes()

        if len(request.camera_name) <= 1:
            self.get_logger().info("Camera name not provided")
            return response

        if request.cam_to_world == 1:
            self.camImage_to_world(request.camera_name, request.stamp)
            response.coords = self.serialized_message
            response.tf_success = True
        elif request.world_to_pixel == 1:
            ok = self.world_to_pixel(
                request.camera_name, request.stamp,
                (request.x, request.y, request.z))
            if ok and self.serialized_message is not None:
                response.coords = self.serialized_message
                response.tf_success = True
            else:
                response.tf_success = False
                response.coords = bytes()
        elif request.pixel_to_world == 1:
            camera_info = self.get_camera_info(request.camera_name)
            if camera_info is None:
                self.get_logger().info(
                    f"No CameraInfo yet for '{request.camera_name}'")
                return response
            if self.latest_cloud is None:
                self.get_logger().info("No LiDAR cloud received yet")
                return response
            try:
                point = self.pixel_to_world(
                    camera_info,
                    request.stamp,
                    self.latest_cloud,
                    float(request.x),
                    float(request.y),
                )
            except TransformException as ex:
                self.get_logger().info(f"pixel_to_world TF failed: {ex}")
                return response
            if point is None:
                self.get_logger().info(
                    f"No LiDAR near pixel ({request.x}, {request.y})")
                return response
            response.coords = np.asarray(point, dtype=np.float64).tobytes()
            response.tf_success = True
        else:
            self.get_logger().info("No transform mode selected")

        return response            

def main(args=None):
    rclpy.init(args=args)
    node = FrameTFService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
