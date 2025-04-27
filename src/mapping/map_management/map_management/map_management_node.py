# Core rclpy dependencies
import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.executors as executors
import rclpy.callback_groups as cbg
import rclpy.qos as qos

# For working with OpenDRIVE maps
from pyOpenDRIVE.OpenDriveMap import PyOpenDriveMap
from pyOpenDRIVE.Lane import PyLane, PyLaneKey
from pyOpenDRIVE.LaneSection import PyLaneSection
from pyOpenDRIVE.Road import PyRoad
from pyOpenDRIVE.RoutingGraph import PyRoutingGraph

# For geometry
import shapely as sh

# For coordinate transforms
import tf2_ros as tf2

# Message types
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from navigator_msgs.srv import SetRoute
from carla_msgs.msg import CarlaWorldInfo

import math

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
    sourceKey: PyLaneKey
    targetKey: PyLaneKey
    sourceLength: float

class MapManagementNode(Node):

    clock_ : Clock | None = None

    map_: PyOpenDriveMap | None = None
    map_wide_tree_: sh.STRtree | None = None
    route_linestring_: sh.LineString = sh.LineString()
    local_route_linestring_: sh.LineString = sh.LineString()
    junction_polys_: list[sh.Polygon] = []
    lane_polys_ : list[tuple[PyLane, sh.LinearRing]] = []
    smoothed_route_msg_: Path | None = None
    route_linestrings_ : list[sh.LineString] = []
    faulty_routing_graph : PyRoutingGraph | None = None 
    road_in_junction_map_ : dict[PyLaneKey, bool] = {}

    def __init__(self):
        super().__init__('map_mangement_node')

        # Set up coordinate transform tools
        self.tf_buffer_ = tf2.Buffer()
        self.tf_listener_ = tf2.TransformListener(self.tf_buffer_, self)

        # Declare & read params
        self.declare_parameter("from_file", False)
        self.declare_parameter("data_path", "/home/nova/navigator/data")
        load_from_file: bool = self.get_parameter("from_file").get_parameter_value().bool_value
        data_path: str = self.get_parameter("data_path").get_parameter_value().string_value

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
        self.clock_subscriber_ = self.create_subscription(Clock, 'clock', self.clockCb, qos.qos_profile_system_default, callback_group=reentrant_group)
        # Start publishing timer for smooth_route
        self.smooth_route_timer_ = self.create_timer(SMOOTH_ROUTE_LS_FREQ, self.publishSmoothRoute, callback_group=reentrant_group)

        if load_from_file:
            self.__load_map_from_file(data_path)
        else:
            self.world_info_sub = self.create_subscription(CarlaWorldInfo, "/carla/world_info", self.worldInfoCb, 10)

        self.drivable_area_grid_pub_timer_ = self.create_timer(GRID_PUBLISH_FREQUENCY, self.drivableAreaGridPubTimerCb)

    def __load_map_from_file(self, data_path: str):
        self.get_logger().info(f"Loading map from {data_path}/maps/campus.xodr")
        self.map_ = PyOpenDriveMap(f"{data_path}/maps/campus.xodr")
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


    def publishSmoothRoute(self):
        # Avoid using clock before it's initialized
        if self.clock_ == None:
            return
        
        clock = self.clock_.clock
        # If we have already created the smoothed route message
        if self.smoothed_route_msg_ != None and len(self.smoothed_route_msg_.poses) > 0:
            self.smoothed_route_msg_.header.stamp = clock
            for pose in self.smoothed_route_msg_.poses:
                pose.header.stamp = clock
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
    
    def laneKeysFromPoint(self, pt : sh.Point, lane_tree : sh.STRtree, lane_polys : list[tuple[PyLane, sh.LinearRing]]) -> list[PyLaneKey]:
        query_results = lane_tree.query(pt, 'intersects')
        lane_keys : list[PyLaneKey] = []

        # Move through the query results, extracting the lane key and appending to the result list
        for result in query_results:
            ring = lane_polys[result][1]
            if sh.within(pt, ring):
                lane_keys.append(lane_polys[result][0].key)

        if len(lane_keys) > 0:
            return lane_keys
        
        # We could not find any lanes intersecting the point
        # We fall back to finding the nearest lane to the point, even if they're not touching
        query_results = lane_tree.query_nearest(pt, all_matches=False)
        lane_keys.append(lane_polys[query_results[0]][0].key)

        dist = sh.distance(pt, lane_tree.geometries[query_results[0]])
        print(f"Lane key does not touch current pos. Distance: {dist}\n")

        return lane_keys
    
    def setPredeterminedRoute(self):
        to_park_keys = [
            PyLaneKey("14", 313.39848024248613, 2),
            PyLaneKey("14", 48.122505395107886, 1),
            PyLaneKey("14", 0.0, 1),
            PyLaneKey("105", 0.0, 1),
            PyLaneKey("0", 0.0, -3),
            PyLaneKey("0", 53.384839499112189, -3),
            PyLaneKey("0", 61.099814596270349, -4),
            PyLaneKey("0", 84.213823743447264, -3),
            PyLaneKey("220", 0.0, -1),
            PyLaneKey("33", 0.0, 3),
            PyLaneKey("4", 0.0, 3),
            PyLaneKey("52", 0.0, 3),
            PyLaneKey("84", 0.0, -3),
            PyLaneKey("62", 0.0, -3),
            PyLaneKey("42", 0.0, -3),
            PyLaneKey("484", 0.0, -1),
            PyLaneKey("30", 0.0, -3),
            PyLaneKey("68", 0.0, -3)
        ]

        route_linestrings_ = []

        for k in to_park_keys:
            print(f"Appending {k.to_string()} to route linestrings.\n")
            key_ls = self.getLaneCenterline(k)
            print(f"Key LS has {len(key_ls)} points\n")
            key_ls_simplified = sh.simplify(key_ls, 2.0)
            route_linestrings_.append(key_ls_simplified)

    def setRoute(self, request : SetRoute.Request, response : SetRoute.Response):
        self.get_logger().info("Received request to set route!")

        if len(request.route_nodes) < 2:
            self.get_logger().error("Service call requires at least two points.")
            response.message = "Service call requires at least two points"
            response.success = False
            return
        elif len(request.route_nodes) > 2:
            self.get_logger().warning("Routing is only supported for two points currently. Remaining points will be ignored.")
        
        if self.map_wide_tree_ == None:
            self.get_logger().error("Cannot set a route before map_wide_tree_ has been initialized.")
            response.message = "Cannot set a route before map_wide_tree_ has been initialized."
            response.success = False
            return
        
        # For the first two points, get their lanekeys
        # TODO: Extend to N points
        from_pt = sh.Point(request.route_nodes[0].x, request.route_nodes[0].y)
        to_pt = sh.Point(request.route_nodes[1].x, request.route_nodes[1].y)
        from_keys = self.laneKeysFromPoint(from_pt, self.map_wide_tree_, self.lane_polys_)
        to_keys = self.laneKeysFromPoint(from_pt, self.map_wide_tree_, self.lane_polys_)
        if len(from_keys) > 1:
            self.get_logger().error("Route start falls within a junction.")
            response.message = f"Route start falls within a junction: {from_keys[0].to_string()}"
            response.success = False
            return
        if len(to_keys) > 1:
            self.get_logger().error("Route end falls within a junction.")
            response.message = "Route end falls within a junction."
            response.success = False
            return
        if len(to_keys) < 1:
            self.get_logger().error("Route end does not fall within a lane.")
            response.message = "Route end does not fall within a lane."
            response.success = False
            return
        
        from_key = from_keys[0]
        to_key = to_keys[0]

        # TODO: Implement A* search here to replace lemon::djikstra
        response.message = "Search is not yet implemented."
        response.success = False
        return

    def publishGrids(self, top_dist : int, bottom_dist : int, side_dist : int, res : float):
        if self.clock_ == None:
            self.get_logger().info("Clock not yet initialized. Drivable area grid is unavailable.")
            return
        if self.map_ == None:
            self.get_logger().info("Map not yet loaded. Drivable area grid is unavailable.")
            return
        
        # Used to calculate function runtime
        # begin = self.clock_.clock # Begin time for timing

        drivable_grid_data = []
        junction_grid_data = []
        # route_dist_grid_data = [] # Commented out in C++ version

        y_min = side_dist * -1
        y_max = side_dist
        x_min = bottom_dist * -1
        x_max = top_dist

        if self.map_wide_tree_ == None or len(self.map_wide_tree_) == 0:
            self.map_wide_tree_ = self.map_.generate_mesh_tree()

        # Get the search region
        vehicle_tf = self.getEgoTf()
        vehicle_pos = vehicle_tf.transform.translation
        range_plus = top_dist * 1.4
        x_min, y_min = vehicle_pos.x - range_plus, vehicle_pos.y - range_plus
        x_max, y_max = vehicle_pos.x + range_plus, vehicle_pos.y + range_plus
        search_region = sh.box(x_min, y_min, x_max, y_max)

        # Find all lanes within the search region
        lane_shapes_in_range: list[sh.Geometry] = []
        query_results = self.map_wide_tree_.query(search_region, 'intersects')
        for i in query_results:
            lane_shapes_in_range.append(self.map_wide_tree_.geometries[i])
        
        has_nearby_lanes = len(lane_shapes_in_range) > 0
        # Warn if there are no nearby lanes.
        # This may simply be because we are testing in an area that is not on the map.
        if not has_nearby_lanes:
            self.get_logger().warn("There are no lane shapes nearby.")

        local_tree: sh.STRtree = sh.STRtree(lane_shapes_in_range, 16)

        area = 0
        height = 0

        goal_pt = sh.Point()
        # goal_is_set = False # Unused, was only used in commented code in C++ version
        q = vehicle_tf.transform.rotation

        if q.z < 0:
            h = abs(2 * math.acos(q.w) - 2 * math.pi)
        else:
            h = 2 * math.acos(q.w)

        if h > math.pi:
            h -= 2 * math.pi

        nearby_junctions: list[sh.Polygon] = []

        for junction_poly in self.junction_polys_:
            vehicle_pos_pt = sh.Point(vehicle_pos.x, vehicle_pos.y)
            dist = sh.distance(vehicle_pos_pt, junction_poly)

            if dist < 40.0:
                nearby_junctions.append(junction_poly)
        
        j = y_min
        while j <= y_max:
            i = x_min
            while i <= x_max:
                i = float(i)
                j = float(j)
                cell_is_drivable = False
                cell_is_in_junction = False

                i_in_map = i * math.cos(h) - j * math.sin(h) + vehicle_pos.x
                j_in_map = j * math.cos(h) + i * math.sin(h) + vehicle_pos.y

                p = sh.Point(i_in_map, j_in_map)
                # This block was commented out:
                query_results = local_tree.query(p, 'contains')
                for geomIndex in query_results:
                    ring = self.lane_polys_[geomIndex][1]
                    lane = self.lane_polys_[geomIndex][0]
                    point_is_within_shape = sh.within(p, ring)
                    if point_is_within_shape:
                        cell_is_in_junction = self.road_in_junction_map_[lane.key]
                        if lane.type == "driving":
                            cell_is_drivable = True
                            break
                # Down to here.
                for poly in nearby_junctions:
                    if sh.within(p, poly):
                        cell_is_in_junction = True
                
                # If no lanes are are nearby, then the vehicle is assumed to be testing on a route
                # area which is not on the campus.xodr map. 
                # In this case, make the entire drivable grid 0 cost. 
                if not has_nearby_lanes:
                    drivable_grid_data.append(0)
                else:
                    drivable_grid_data.append(0 if cell_is_drivable else 100)

                junction_grid_data.append(100 if cell_is_in_junction else 0)

                area += 1
                i += res
            height += 1
            j += res
        
        clock = self.clock_.clock
        drivable_area_grid = OccupancyGrid()
        drivable_area_grid.data = drivable_grid_data
        drivable_area_grid.header.frame_id = "base_link"
        drivable_area_grid.header.stamp = clock

        junction_grid = OccupancyGrid()
        junction_grid.data = junction_grid_data
        junction_grid.header.frame_id = "base_link"
        junction_grid.header.stamp = clock

        #route_dist_grid.data = route_dist_grid_data;
        #route_dist_grid.header.frame_id = "base_link";
        #route_dist_grid.header.stamp = clock;

        grid_info = MapMetaData()
        grid_info.width = area / height
        grid_info.height = height
        grid_info.map_load_time = clock
        grid_info.resolution = res
        grid_info.origin.position.x = x_min
        grid_info.origin.position.y = y_min

        # Set the grids' metadata
        drivable_area_grid.info = grid_info
        junction_grid.info = grid_info
        #route_dist_grid.info = grid_info

        # Output function runtime
        self.drivable_grid_pub_.publish(drivable_area_grid)
        self.junction_grid_pub_.publish(junction_grid)
        #self.route_dist_grid_pub_.publish(route_dist_grid) # Route distance grid

        goal_pose = PoseStamped()
        goal_pose.pose.position.x = goal_pt.x
        goal_pose.pose.position.y = goal_pt.y
        goal_pose.header.frame_id = "base_link"
        goal_pose.header.stamp = clock
        self.goal_pose_pub_.publish(goal_pose)

        # end = self.clock_.clock # End time for timing
                
    # Return the (x,y) point at the start of a lane's outer border.
    def getLaneStart(self, key : PyLaneKey):
        current_road = self.map_.id_to_road[key.road_id]
        current_lsec = current_road.s_to_lanesection[key.lanesection_s0]
        current_lane = current_lsec.id_to_lane[key.lane_id]

        if key.lane_id < 0:
            s_start = key.lanesection_s0
        else:
            s_start = current_road.get_lanesection_end(key.lanesection_s0)
        
        t = current_lane.outer_border.get(s_start)
        start_vec = current_road.get_surface_pt(s_start, t)
        start_xy = sh.Point(start_vec.array[0], start_vec.array[1])

        return start_xy

    """
    Given a key, find true successors: immediately connected lanes
    ahead of the given one with respect to traffic flow.
    
    Assuming right-hand driving rules, we treat the "start" of a lane as having:
    - Either s = lanesection_s0 and ID < 0, OR
    - s = lanesection_s_max and ID > 0.
    
    Recall that lanesections are identified by their start and end s values.
    """
    def getTrueSuccessors(self, key : PyLaneKey) -> list[PyLaneKey]:
        results : list[PyLaneKey] = []

        # We assume that each lane's listed successors and predecessors could be
        # actual successfors, and this function verifies or refutes each one,
        # returning the verified ones.

        if self.faulty_routing_graph == None:
            self.faulty_routing_graph = self.map_.get_routing_graph()
            self.get_logger().error("Routing graph was empty!")
        
        # Get the current lane's endpoint (x,y)
        current_road = self.map_.id_to_road[key.road_id]
        current_lsec = current_road.s_to_lanesection[key.lanesection_s0]
        current_lane = current_lsec.id_to_lane[key.lane_id]

        if key.lane_id > 0:
            s_end = key.lanesection_s0
        else:
            s_end = current_road.get_lanesection_end(key.lanesection_s0)
        
        t = current_lane.outer_border.get(s_end)
        end_vec = current_road.get_surface_pt(s_end, t)
        end_xy = sh.Point(end_vec.array[0], end_vec.array[1])

        # TODO: Need full routing graph implementation in wrapper for this!!!!
        predecessors = self.faulty_routing_graph.get_lane_predecessors(key)

        # For each predecessor, get their lane's START (x,y)
        for predecessor in predecessors:
            start_xy = self.getLaneStart(predecessor)
            dist = sh.distance(end_xy, start_xy)
            if dist  < 0.05:
                results.append(predecessor)
        
        successors = self.faulty_routing_graph.get_lane_successors(key)

        # For each successor, get their lane's START (x,y)
        for successor in successors:
            start_xy = self.getLaneStart(successor)
            dist = sh.distance(end_xy, start_xy)
            if dist  < 0.05:
                results.append(successor)

        return results
            
    
    def buildTrueRoutingGraph(self):
        if len(self.lane_polys_) < 1 or self.map_ == None:
            self.get_logger().error("Map not yet loaded. Routing graph could not be built.")
            return
        
        if self.faulty_routing_graph == None:
            self.faulty_routing_graph = self.map_.get_routing_graph()
            self.get_logger().info("Built rough routing graph.")

        # TODO: Replace lemon here!!!
        raise NotImplementedError()

    #def recursiveSearch(self, predecessors : list[PyLaneKey], target : PyLaneKey):
    #    # TODO
    #    pass

    #def calculateRoute(self, start : PyLaneKey, end : PyLaneKey) -> list[PyLaneKey]:
    #    # TODO
    #    pass

    # Caches the latest clock from CARLA.
    def clockCb(self, clock: Clock):
        self.clock_ = clock

    def getEgoTf(self) -> tf2.TransformStamped:
        t: tf2.TransformStamped
        try:
            t = self.tf_buffer_.lookup_transform("map", "base_link", 0)
        except:
            self.get_logger().info(f"Could not get base_link->map tf. This will republish every 5 seconds.", throttle_duration_sec=5)
        return t

    def drivableAreaGridPubTimerCb(self):
        self.publishGrids(40, 20, 30, 0.4)

    #def add_edge(adj : list[list[int]], src : int, dest : int):
    #    # TODO: Unused in C++ version
    #    pass

    #def getSmoothSection(full_route : sh.LineString, ego : sh.Point, start_idx : int, end_idx : int) -> sh.LineString:
    #    # TODO: Unused in C++ version
    #    pass

    def getLaneCenterline(self, key : PyLaneKey):
        road = self.map_.id_to_road[key.road_id]
        lsec = road.s_to_lanesection[key.lanesection_s0]
        lane = lsec.id_to_lane[key.lane_id]

        outer_border = road.get_lane_border_line(lane, 1.0, True)
        inner_border = road.get_lane_border_line(lane, 1.0, False)

        centerline_pts = []

        for i in range(len(outer_border)):
            outer_pt = outer_border.array[i].array
            inner_pt = inner_border.array[i].array
            center_pt = sh.Point((outer_pt[0] + inner_pt[0]) / 2, (outer_pt[1] + inner_pt[1]) / 2)
            centerline_pts.append(center_pt)

        return sh.LineString(centerline_pts)

    #def getReorientedRoute(centerlines : list[sh.LineString]) -> sh.LineString:
    #    # TODO: Unused in C++ version
    #    pass

    #def updateRouteWaypoints(self, msg : Path):
    #    # TODO: Unused in C++ version
    #   pass

    #def getWaypointsInROI(waypoints : sh.LineString, tree : sh.STRtree, ego_pos : sh.Point) -> RoiIndices:
    #    # TODO: Unused in C++ version
    #    pass

    #def getLaneKeysFromIndices(indices : RoiIndices, waypoints : sh.LineString, lane_tree : sh.STRtree, lane_polys : list[tuple[PyLane, sh.LinearRing]]) -> list[PyLaneKey]:
    #    # TODO: Unused in C++ version
    #    pass

    """
    NOTE: CURRENTLY UNUSED

    Given a destination point (x,y), set our route linestring
    to be the continuous centerlines of the lanes connecting
    the current location to the destination. Should be fast.
    """
    #def updateRouteGivenDestination(self, destination : sh.Point):
    #    # TODO: Unused in C++ version
    #    pass

    def getJunctionMap(self, lane_polys : list[tuple[PyLane, sh.LinearRing]]) -> dict[PyLaneKey, bool]:
        m = {}
        for pair in lane_polys:
            lane = pair[0]
            road = self.map_.id_to_road[lane.key.road_id]
            if road.junction != "-1" and lane.type == "driving":
                m[lane.key] = True
            else:
                m[lane.key] = False
        
        return m

    # When map data is received from a CarlaWorldInfo message, load and process it
    def worldInfoCb(self, msg: CarlaWorldInfo):
        if self.map_ != None:
            return # Map is already loaded. No need to continue.
        
        if msg.opendrive == "":
            self.get_logger().info("Received empty map string from world_info topic. Waiting for real data.")
            return
        
        # Ask pyOpenDRIVE to parse the map string
        self.map_ = PyOpenDriveMap(msg.opendrive, True)
        # Get lane polygons as pairs (Lane object, ring polygon)
        self.lane_polys_ = self.map_.get_lane_polygons(1.0, False)

        self.road_in_junction_map_ = self.getJunctionMap(self.lane_polys_)

        self.get_logger().info(f"Loaded {msg.map_name}")

        self.buildTrueRoutingGraph()


def main():
    rclpy.init()
    executor = executors.MultiThreadedExecutor(NUM_THREADS)
    executor.add_node(MapManagementNode())
    executor.spin()

if __name__ == '__main__':
    main()