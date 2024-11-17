# Core rclpy dependencies
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.executors as executors
import rclpy.callback_groups as cbg
import rclpy.qos as qos

# For working with OpenDRIVE maps
import pyopendrive

# For geometry
import shapely as sh

# For coordinate transforms
import tf2_ros as tf2

# Message types
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from navigator_msgs.srv import SetRoute
from carla_msgs.msg import CarlaWorldInfo

# Number of threads to use for the multithreaded executor, None = All threads
NUM_THREADS: int = None
# Smoothed route publishing frequency in seconds
SMOOTH_ROUTE_LS_FREQ: float = 1
# Drivable area grid publish frequency in seconds
GRID_PUBLISH_FREQUENCY: float = 3

# Mutually exclusive callback group for MTE; prevents callbacks in group from being parallelized 
mutex_group = cbg.MutuallyExclusiveCallbackGroup()
# Reentrant callback group; allows callbacks to be parallelized
reentrant_group = cbg.ReentrantCallbackGroup()

class RoiIndices:
    start: int = -1
    center: int = -1
    end: int = -1

class Arc:
    sourceKey: pyopendrive.LaneKey
    targetKey: pyopendrive.LaneKey
    sourceLength: float

class MapManagementNode(Node):

    map_: pyopendrive.OpenDriveMap
    map_wide_tree_: sh.STRtree
    route_linestring_: sh.LineString
    local_route_linestring_: sh.LineString
    junction_polys_: list[sh.Polygon]
    smoothed_route_msg_: Path

    def __init__(self):
        super().__init__('map_mangement_node')

        # Set up coordinate transform tools
        self.tf_buffer_ = tf2.Buffer(self.get_clock())
        self.tf_listener_ = tf2.TransformListener(self.tf_buffer_, self)

        # Declare & read params
        self.declare_parameter("from_file", False)
        self.declare_parameter("data_path", "/home/nova/navigator/data")
        load_from_file: bool = self.get_parameter("from_file")
        data_path: str = self.get_parameter("data_path")

        # Set up publishers
        self.drivable_grid_pub_ = self.create_publisher(OccupancyGrid, "/grid/drivable", qos.qos_profile_sensor_data)
        self.junction_grid_pub_ = self.create_publisher(OccupancyGrid, "/grid/junction", qos.qos_profile_sensor_data)
        self.route_path_pub_ = self.create_publisher(Path, "/planning/smoothed_route", 10)
        self.goal_pose_pub_ = self.create_publisher(PoseStamped, "/planning/goal_pose", 1)

        # TODO: Actually implement this? Original C++ impl. was commented out.
        # clicked_point_sub_ = this->create_subscription<PointStamped>("/clicked_point", 1, bind(&MapManagementNode::clickedPointCb, this, std::placeholders::_1));

        # Set up "set_route" service
        self.route_set_service_ = self.create_service(SetRoute, "set_route", self.setRoute)

        # Set up subscriptions
        self.clock_subscriber_ = self.create_subscription(Clock, 'clock', self.clock_callback, qos.qos_profile_system_default, callback_group=reentrant_group)
        # Start publishing timer for smooth_route
        self.smooth_route_timer_ = self.create_timer(SMOOTH_ROUTE_LS_FREQ, self.publishSmoothRoute, callback_group=reentrant_group)

        if load_from_file:
            self.__load_map_from_file(data_path)
        else:
            self.world_info_sub = self.create_subscription(CarlaWorldInfo, "/carla/world_info", self.worldInfoCb, 10)

        self.drivable_area_grid_pub_timer_ = self.create_timer(GRID_PUBLISH_FREQUENCY, self.drivableAreaGridPubTimerCb)

    def __load_map_from_file(self, data_path: str):
        self.get_logger().info(f"Loading map from {data_path}/maps/campus.xodr")
        self.map_ = pyopendrive.OpenDriveMap(f"{data_path}/maps/campus.xodr")
        self.lane_polys_ = self.map_.get_lane_polygons(1.0, False)

        self.get_logger().info(f"Map loaded with {len(self.map_.get_roads())} roads")

        if (self.map_wide_tree_ == None):
            self.get_logger().info("map_wide_tree_ was empty. Generating.")
            self.map_wide_tree_ = self.map_.generate_mesh_tree()

        self.buildTrueRoutingGraph()

        self.setPredeterminedRoute()

        # Temporary linestring from file
        with open(f"{data_path}/maps/route_wkt.txt") as ifs:
            self.route_linestring_ = sh.from_wkt(ifs)

        self.get_logger().info(f"Route LS has {len(self.route_linestring_.coords)} pts\n")

        # Read junctions from file
        junction_mps: sh.MultiPolygon
        with open(f"{data_path}/maps/junction_wkt.txt") as ifs:
            junction_mps = sh.from_wkt(ifs)

        self.junction_polys_.clear()
        self.junction_polys_.extend(junction_mps.geoms)

        self.get_logger().info(f"Added {len(self.junction_polys_)} junction polygons\n")

        self.get_logger().info(f"Route LS now has {len(self.route_linestring_.coords)} pts\n")

        return

    # Caches the latest clock from CARLA.
    def clock_callback(self, clock: Clock):
        self.clock_ = clock

    def worldInfoCb(self, msg: CarlaWorldInfo):
        # TODO
        pass

    def drivableAreaGridPubTimerCb(self):
        # TODO
        pass

    """
    * Given a global (possibly large) route linestring:
    * 1. Find the point closest to the car.
    * 2. Select the point following it in the linestring.
    * 3. For this point and the following points, append them until the current point's
    * distance to the car is beyond some max constant
    """
    def updateLocalRouteLinestring(self):
        MAX_DISTANCE: float = 50 # meters
        min_distance: float = 999999.9

        ego_trans1 = self.getEgoTf().transform.translation
        ego_pos: sh.Point = sh.Point(ego_trans1.x, ego_trans1.y)

        min_idx: int = 0
        for i in range(len(self.route_linestring_.coords)):
            pt: sh.Point = sh.Point(self.route_linestring_.coords[i])
            dist: float = pt.distance(ego_pos)
            if dist < min_distance:
                min_distance = dist
                min_idx = i
        
        if min_idx > 0:
            min_idx -= 1

        filtered_coords: list[tuple[float, float]] = []
        for coord in self.route_linestring_.coords[min_idx:]:
            pt: sh.Point = sh.Point(coord)
            if pt.distance(ego_pos) > MAX_DISTANCE:
                break
            filtered_coords.append(coord)

        self.local_route_linestring_ = sh.LineString(filtered_coords)


    def getEgoTf(self) -> tf2.TransformStamped:
        t: tf2.TransformStamped
        try:
            t = self.tf_buffer_.lookup_transform("map", "base_link", 0)
        except:
            self.get_logger().info(f"Could not get base_link->map tf. This will republish every 5 seconds.", throttle_duration_sec=5)
        return t


    def publishSmoothRoute(self):
        # Avoid using clock before it's initialized
        if self.clock_ == None:
            return
        
        clock = self.clock_.clock
        # If we have already created the smoothed route message
        if self.smoothed_route_msg_ != None and len(self.smoothed_route_msg_.poses) > 0:
            self.smoothed_route_msg_.header.stamp = clock
            for i in range(len(self.smoothed_route_msg_.poses)):
                self.smoothed_route_msg_.poses[i].header.stamp = clock
            self.route_path_pub_.publish(self.smoothed_route_msg_)
        # If the route has been defined, but the message hasn't been made
        elif len(self.route_linestring_.coords) > 0:
            self.smoothed_route_msg_.header.frame_id = "map"
            self.smoothed_route_msg_.header.stamp = clock
            for i in range(len(self.route_linestring_.coords)):
                pose: PoseStamped = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = clock
                pose.pose.position.x, pose.pose.position.y = self.route_linestring_.coords[i]
                self.smoothed_route_msg_.poses[i] = pose
            self.route_path_pub_.publish(self.smoothed_route_msg_)
    
    def setRoute(self, request, response):
        # TODO
        pass
    
    def buildTrueRoutingGraph():
        # TODO
        pass

    def setPredeterminedRoute():
        # TODO
        pass

def main():
    rclpy.init()
    executor = executors.MultiThreadedExecutor(NUM_THREADS)
    executor.add_node(MapManagementNode())
    executor.spin()

if __name__ == '__main__':
    main()