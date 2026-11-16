#!/usr/bin/env python3
"""
Package: pedestrian_crossing
File: pedestrian_crossing_node.py
Author: Anirudh Devatha

Detects pedestrians on or approaching the vehicle's planned route and
publishes a TTC-based status signal for the behavior layer.

Subscribes:
  /tracked/objects3d                   (navigator_msgs/Object3DArray)  -- required
  /planning/path                       (nav_msgs/Path)                 -- required
  /pedestrians                         (navigator_msgs/PedestrianInfoDetections) -- optional
  /carla/hero/rgb_front/camera_info    (sensor_msgs/CameraInfo)        -- optional

Publishes:
  /pedestrian_status  (navigator_msgs/PedestrianStatusArray)

This node owns ROS I/O only. The classification itself lives in
crossing_logic.py, which has no ROS dependencies and is unit tested directly
-- the same split navigator_lane_change uses.

Frames
------
All geometry runs in the frame of /planning/path, which the current path
planner publishes as base_link (path_planner_node.py). Tracked objects arrive
in whatever frame lidar_detection stamped them with (mot3d_node passes the
detection frame straight through), so if that differs from the path frame the
objects are transformed via TF before any corridor math runs. In the common
case both are base_link and no TF lookup happens at all.

Optional camera layer
---------------------
/pedestrians (from pedestrian_intent_to_enter_road) contributes a gaze and
distance-to-road cue that can promote UNCERTAIN to INTENT_TO_CROSS. That node
is not enabled in the default perception launch, and its `distance` field is
inert unless something publishes /segmentation_mask, so the whole camera
layer is treated as optional enrichment: every pedestrian is fully classified
from the 3D tracker alone, and `camera_corroborated` on the output records
whether the camera actually contributed. Set `use_camera_layer:=false` to
skip the subscriptions entirely.
"""

import math
import threading
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException

from navigator_msgs.msg import (
    Object3DArray,
    PedestrianInfoDetections,
    PedestrianStatus,
    PedestrianStatusArray,
)

from pedestrian_crossing.crossing_logic import (
    APPROACH_ZONE_EXTRA,
    LOOKAHEAD_DISTANCE,
    PATH_CORRIDOR_HALF_WIDTH,
    PedestrianStateHandler,
    Polygon,
    build_corridor_polygon,
    nearest_path_point,
    point_in_polygon,
)

try:
    import image_geometry
except ImportError:  # pragma: no cover - only needed for the optional camera layer
    image_geometry = None


# Max pixel distance to match a tracked 3D object to a camera detection.
CAMERA_MATCH_THRESHOLD = 80.0

# `distance` sentinel emitted by pedestrian_intent_to_enter_road when it has
# no segmentation mask to measure against. Treated as "unknown", not as a
# real measurement.
CAMERA_DISTANCE_SENTINEL = 1e6

# A camera detection closer to the road than this counts as road-engaged (m).
CAMERA_ROAD_PROXIMITY = 5.0

# Object3D.PEDESTRIAN
PEDESTRIAN_LABEL = 0


class PedestrianCrossingNode(Node):
    """
    Combines 3D tracking, path geometry, and (optionally) camera detections
    into a per-pedestrian crossing status for the behavior layer.
    """

    def __init__(self):
        super().__init__('pedestrian_crossing_node')

        # -- parameters ----------------------------------------------------
        self.declare_parameter('tracked_objects_topic', '/tracked/objects3d')
        self.declare_parameter('path_topic', '/planning/path')
        self.declare_parameter('camera_detections_topic', '/pedestrians')
        self.declare_parameter(
            'camera_info_topic', '/carla/hero/rgb_front/camera_info')
        self.declare_parameter('camera_frame', 'hero/rgb_front')
        self.declare_parameter('status_topic', '/pedestrian_status')
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('corridor_half_width', PATH_CORRIDOR_HALF_WIDTH)
        self.declare_parameter('lookahead_distance', LOOKAHEAD_DISTANCE)
        self.declare_parameter('approach_zone_extra', APPROACH_ZONE_EXTRA)
        self.declare_parameter('input_timeout', 2.0)
        self.declare_parameter('use_camera_layer', True)

        self.tracked_objects_topic = self.get_parameter('tracked_objects_topic').value
        self.path_topic = self.get_parameter('path_topic').value
        self.camera_detections_topic = self.get_parameter('camera_detections_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.status_topic = self.get_parameter('status_topic').value
        self.update_rate = self.get_parameter('update_rate').value
        self.corridor_half_width = self.get_parameter('corridor_half_width').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.approach_zone_extra = self.get_parameter('approach_zone_extra').value
        self.input_timeout = self.get_parameter('input_timeout').value
        self.use_camera_layer = self.get_parameter('use_camera_layer').value

        # -- state ---------------------------------------------------------
        self._lock = threading.Lock()

        self.pedestrian_handlers: Dict[int, PedestrianStateHandler] = {}

        self.latest_tracked_msg: Optional[Object3DArray] = None
        self.current_path: Optional[Path] = None
        self.camera_pedestrians: Optional[PedestrianInfoDetections] = None

        # Wall-clock receipt times, used to tell "never arrived" apart from
        # "arrived and then stopped".
        self.tracker_last_rx: Optional[float] = None
        self.path_last_rx: Optional[float] = None

        self._failsafe_active = False

        self.cam_model = None
        self._camera_warned = False

        # -- TF ------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # -- subscriptions -------------------------------------------------
        self.tracker_sub = self.create_subscription(
            Object3DArray, self.tracked_objects_topic, self.tracker_callback, 10)
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self.path_callback, 10)

        if self.use_camera_layer:
            self.pedestrian_sub = self.create_subscription(
                PedestrianInfoDetections, self.camera_detections_topic,
                self.pedestrian_callback, 10)
            self.cam_info_sub = self.create_subscription(
                CameraInfo, self.camera_info_topic, self.cam_info_callback, 10)

        # -- publisher -----------------------------------------------------
        self.status_pub = self.create_publisher(
            PedestrianStatusArray, self.status_topic, 10)

        self.create_timer(1.0 / self.update_rate, self.update)

        self.get_logger().info(
            f'PedestrianCrossingNode started. '
            f'tracker={self.tracked_objects_topic} path={self.path_topic} '
            f'camera_layer={"on" if self.use_camera_layer else "off"}')

    # -- callbacks ---------------------------------------------------------

    def tracker_callback(self, msg: Object3DArray):
        with self._lock:
            self.latest_tracked_msg = msg
            self.tracker_last_rx = self._now()

    def path_callback(self, msg: Path):
        with self._lock:
            self.current_path = msg
            self.path_last_rx = self._now()

    def pedestrian_callback(self, msg: PedestrianInfoDetections):
        with self._lock:
            self.camera_pedestrians = msg

    def cam_info_callback(self, msg: CameraInfo):
        if self.cam_model is not None:
            return
        if image_geometry is None:
            if not self._camera_warned:
                self.get_logger().warning(
                    'image_geometry unavailable; camera layer inactive.')
                self._camera_warned = True
            return
        model = image_geometry.PinholeCameraModel()
        model.fromCameraInfo(msg)
        self.cam_model = model
        self.get_logger().info('Camera model loaded; camera layer active.')

    # -- main loop ---------------------------------------------------------

    def update(self):
        """Classify every tracked pedestrian and publish the result."""
        with self._lock:
            tracked = self.latest_tracked_msg
            path = self.current_path
            camera = self.camera_pedestrians
            tracker_rx = self.tracker_last_rx
            path_rx = self.path_last_rx

        now = self._now()

        # Nothing has ever arrived: the node is inert, not alarmed. Publishing
        # STOP here would put a freshly built graph with no perception running
        # into permanent emergency braking.
        if tracker_rx is None or path_rx is None:
            missing = []
            if tracker_rx is None:
                missing.append(self.tracked_objects_topic)
            if path_rx is None:
                missing.append(self.path_topic)
            self.get_logger().info(
                f'Waiting for first message on: {", ".join(missing)}',
                throttle_duration_sec=10.0)
            self._publish_empty()
            return

        # Inputs were live and have since dropped out. That is a real fault,
        # and the vehicle should stop rather than coast on stale classifications.
        tracker_age = now - tracker_rx
        path_age = now - path_rx
        if tracker_age > self.input_timeout or path_age > self.input_timeout:
            stale = []
            if tracker_age > self.input_timeout:
                stale.append(f'{self.tracked_objects_topic} ({tracker_age:.1f}s)')
            if path_age > self.input_timeout:
                stale.append(f'{self.path_topic} ({path_age:.1f}s)')
            self._publish_failsafe(f'input stale: {", ".join(stale)}')
            return

        if self._failsafe_active:
            self.get_logger().info('Inputs recovered; leaving fail-safe.')
            self._failsafe_active = False

        path_frame = path.header.frame_id or 'base_link'
        waypoints = [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]

        corridor = build_corridor_polygon(
            waypoints, self.corridor_half_width, self.lookahead_distance)
        approach_corridor = build_corridor_polygon(
            waypoints,
            self.corridor_half_width + self.approach_zone_extra,
            self.lookahead_distance)

        if not corridor:
            self.get_logger().warning(
                'Path too short to build a corridor; publishing empty status.',
                throttle_duration_sec=5.0)
            self._publish_empty(frame_id=path_frame)
            return

        positions = self._pedestrian_positions_in(tracked, path_frame)
        stamp = self._stamp_seconds(tracked.header.stamp) or now

        seen_ids = set()
        for ped_id, (x, y, z) in positions.items():
            seen_ids.add(ped_id)

            handler = self.pedestrian_handlers.get(ped_id)
            if handler is None:
                handler = PedestrianStateHandler(ped_id)
                self.pedestrian_handlers[ped_id] = handler

            facing_road, dist_to_road, corroborated = self._match_camera_detection(
                camera, x, y, z)
            nearest_x, nearest_y = nearest_path_point(waypoints, x, y)

            handler.update(
                x=x,
                y=y,
                stamp=stamp,
                in_corridor=point_in_polygon(x, y, corridor),
                in_approach_zone=point_in_polygon(x, y, approach_corridor),
                facing_road=facing_road,
                distance_to_road=dist_to_road,
                camera_corroborated=corroborated,
                nearest_path_x=nearest_x,
                nearest_path_y=nearest_y,
            )

        for ped_id, handler in list(self.pedestrian_handlers.items()):
            if ped_id in seen_ids:
                continue
            handler.tick_unseen()
            if handler.is_stale():
                del self.pedestrian_handlers[ped_id]

        self._publish_status(corridor, seen_ids, path_frame)

    # -- tracker ingestion -------------------------------------------------

    def _pedestrian_positions_in(
        self, tracked: Object3DArray, target_frame: str
    ) -> Dict[int, Tuple[float, float, float]]:
        """
        Extract pedestrian positions from the tracker, expressed in
        `target_frame`.

        BoundingBox3D carries `coordinates` as
        (x, y, z, x_size, y_size, z_size, yaw). There is no `center` member on
        this message type.
        """
        source_frame = tracked.header.frame_id or target_frame

        transform = None
        if source_frame != target_frame:
            # One lookup per frame, not per pedestrian.
            transform = self._lookup_transform(target_frame, source_frame)
            if transform is None:
                # Without a usable transform the corridor test would be
                # meaningless, so skip this frame rather than guess.
                return {}

        positions: Dict[int, Tuple[float, float, float]] = {}
        for obj in tracked.objects:
            if obj.label != PEDESTRIAN_LABEL:
                continue

            coords = obj.bounding_box.coordinates
            if len(coords) < 3:
                continue

            x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
            if transform is not None:
                x, y, z = transform(x, y, z)

            positions[int(obj.id)] = (x, y, z)

        return positions

    def _lookup_transform(self, target_frame: str, source_frame: str):
        """Return a callable applying source_frame -> target_frame, or None."""
        try:
            t = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time())
        except TransformException as e:
            self.get_logger().warning(
                f'TF {source_frame} -> {target_frame} unavailable: {e}',
                throttle_duration_sec=5.0)
            return None

        q = t.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w])
        trans = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])

        def apply(x: float, y: float, z: float) -> Tuple[float, float, float]:
            p = rot.apply(np.array([x, y, z])) + trans
            return float(p[0]), float(p[1]), float(p[2])

        return apply

    # -- optional camera layer ---------------------------------------------

    def _world_to_pixel(
        self, x: float, y: float, z: float
    ) -> Optional[Tuple[float, float]]:
        """
        Project a point in the path frame into camera pixel coordinates,
        following the same two steps as image_projection_node:

            1. transform the point into the camera frame via TF
            2. apply the 3x4 projection matrix and divide through by depth

        Returns None when the camera model is not loaded, TF is unavailable,
        or the point falls behind the image plane.
        """
        if self.cam_model is None or self.current_path is None:
            return None

        path_frame = self.current_path.header.frame_id or 'base_link'
        transform = self._lookup_transform(self.camera_frame, path_frame)
        if transform is None:
            return None

        cx, cy, cz = transform(x, y, z)
        if cz <= 0:  # Behind the camera.
            return None

        P = np.asarray(self.cam_model.projectionMatrix())
        pix = P @ np.array([cx, cy, cz, 1.0])
        if abs(pix[2]) < 1e-9:
            return None

        return float(pix[0] / pix[2]), float(pix[1] / pix[2])

    def _match_camera_detection(
        self,
        camera: Optional[PedestrianInfoDetections],
        x: float,
        y: float,
        z: float,
    ) -> Tuple[bool, Optional[float], bool]:
        """
        Try to match a tracked pedestrian to a camera detection.

        Matching happens in pixel space: the tracked 3D position is projected
        into the image and compared against detection bounding-box centers.
        PedestrianInfo.x/.y are already box centers -- pedestrian_intent_to_
        enter_road fills them from YOLO's xywh, which is center-based -- so no
        corner-to-center correction is applied.

        Every failure path returns "no camera contribution" rather than a
        guess. The alternative, comparing a metric position against a pixel
        coordinate, produces a number with no meaning.

        Returns:
            (facing_road, distance_to_road, camera_corroborated)
        """
        if not self.use_camera_layer or camera is None or not camera.pedestrians:
            return False, None, False

        proj = self._world_to_pixel(x, y, z)
        if proj is None:
            return False, None, False

        proj_u, proj_v = proj

        best_match = None
        best_dist = float('inf')
        for cam_ped in camera.pedestrians:
            pixel_dist = math.hypot(cam_ped.x - proj_u, cam_ped.y - proj_v)
            if pixel_dist < best_dist:
                best_dist = pixel_dist
                best_match = cam_ped

        if best_match is None or best_dist > CAMERA_MATCH_THRESHOLD:
            return False, None, False

        # pedestrian_intent_to_enter_road returns CAMERA_DISTANCE_SENTINEL
        # when it has no segmentation mask to measure against. Nothing in the
        # current stack publishes /segmentation_mask, so treat that as
        # "matched, distance unknown" rather than "very far from the road".
        distance = float(best_match.distance)
        if distance >= CAMERA_DISTANCE_SENTINEL:
            return True, None, True

        # The upstream node only publishes pedestrians it has already judged
        # to be facing the road, so a match is itself the gaze signal; the
        # distance refines how close to the road they are.
        return distance <= CAMERA_ROAD_PROXIMITY, distance, True

    # -- publishing --------------------------------------------------------

    def _publish_status(self, corridor: Polygon, seen_ids: set, frame_id: str):
        """Publish one entry per pedestrian observed this frame."""
        status_array = PedestrianStatusArray()
        status_array.header.stamp = self.get_clock().now().to_msg()
        status_array.header.frame_id = frame_id

        for ped_id in sorted(seen_ids):
            handler = self.pedestrian_handlers.get(ped_id)
            if handler is None:
                continue

            ttc, status, trajectory = handler.compute_ttc_and_signal(corridor)

            msg = PedestrianStatus()
            msg.pedestrian_id = ped_id
            msg.status = status
            msg.intent = int(handler.state)
            msg.ttc = float(ttc)
            msg.camera_corroborated = handler.camera_corroborated
            msg.predicted_trajectory = [
                Point(x=float(px), y=float(py), z=0.0) for px, py in trajectory
            ]
            status_array.statuses.append(msg)

        self.status_pub.publish(status_array)

        if status_array.statuses:
            worst = max(s.status for s in status_array.statuses)
            name = {0: 'CLEAR', 1: 'SLOW', 2: 'STOP'}.get(worst, '?')
            live_ttcs = [s.ttc for s in status_array.statuses if s.ttc > 0]
            ttc_str = f'{min(live_ttcs):.1f}s' if live_ttcs else 'none'
            self.get_logger().debug(
                f'Ped status: {name} | TTC: {ttc_str} '
                f'| {len(status_array.statuses)} tracked')

    def _publish_empty(self, frame_id: str = 'base_link'):
        """Publish an empty array: no pedestrians, no constraint."""
        status_array = PedestrianStatusArray()
        status_array.header.stamp = self.get_clock().now().to_msg()
        status_array.header.frame_id = frame_id
        self.status_pub.publish(status_array)

    def _publish_failsafe(self, reason: str):
        """Publish a synthetic STOP because a required input dropped out."""
        if not self._failsafe_active:
            self.get_logger().error(f'Entering fail-safe STOP: {reason}')
            self._failsafe_active = True
        else:
            self.get_logger().warning(
                f'Fail-safe STOP: {reason}', throttle_duration_sec=5.0)

        status_array = PedestrianStatusArray()
        status_array.header.stamp = self.get_clock().now().to_msg()
        status_array.header.frame_id = 'base_link'

        msg = PedestrianStatus()
        msg.pedestrian_id = PedestrianStatus.FAILSAFE_ID
        msg.status = PedestrianStatus.STOP
        msg.intent = PedestrianStatus.IRRELEVANT
        msg.ttc = -1.0
        msg.camera_corroborated = False
        status_array.statuses.append(msg)

        self.status_pub.publish(status_array)

    # -- time --------------------------------------------------------------

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_seconds(stamp) -> Optional[float]:
        secs = stamp.sec + stamp.nanosec * 1e-9
        return secs if secs > 0 else None


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PedestrianCrossingNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
