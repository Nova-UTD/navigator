/*
 * Package:   map_management
 * Filename:  MapManager.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"

#include "map_management/MapManager.hpp"

#include <algorithm>
#include <chrono>
#include <list>

#include <fstream>

using namespace navigator::planning;

struct RoiIndices
{
    int start = -1;
    int center = -1;
    int end = -1;
};

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    // Publishers and subscribers
    drivable_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/drivable", 10);
    junction_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/junction", 10);
    route_dist_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/route_distance", 10);
    route_path_pub_ = this->create_publisher<Path>("/planning/smooth_route", 10);
    traffic_light_points_pub_ = this->create_publisher<PolygonStamped>("/traffic_light_points", 10);
    goal_pose_pub_ = this->create_publisher<PoseStamped>("/planning/goal_pose", 1);
    route_progress_pub_ = this->create_publisher<std_msgs::msg::Float32>("/route_progress", 1);

    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    rough_path_sub_ = this->create_subscription<Path>("/planning/rough_route", 10, bind(&MapManagementNode::updateRouteWaypoints, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));

    drivable_area_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::drivableAreaGridPubTimerCb, this));
    route_timer_ = this->create_wall_timer(ROUTE_PUBLISH_FREQUENCY, bind(&MapManagementNode::publishRefinedRoute, this));

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

/**
 * @brief Returns an OccupancyGrid for lanes of type 'driving'
 *
 * General steps:
 * 1. Query the map-wide R-tree to find all lanes within range (https://www.boost.org/doc/libs/1_72_0/libs/geometry/doc/html/geometry/spatial_indexes/introduction.html)
 * 2. Create a second, local R-tree and insert all nearby lanes, which were found from (1)
 * 3. For each row j and column i, query the local R-tree to see if that cell (i,j) is within a lane's bounding box
 *  a.If yes, find out if it is within the actual lane, not just its bounding box. R-trees only calculate for bounding boxes.
 *      i. If yes again, the cell is truly occupied. Append '100' ("occupied") to OccupancyGrid. Otherwise '0'.
 * 4. Set OccupancyGrid metadata and return.
 *
 * @param center Center of the grid
 * @param top_dist Distance from car to top edge
 * @param bottom_dist Distance from car to bottom edge
 * @param side_dist Distance from car to left and right edges
 * @param res Side length of grid cells (meters)
 * @return OccupancyGrid
 */
void MapManagementNode::publishGrids(int top_dist, int bottom_dist, int side_dist, float res)
{
    if (this->map_ == nullptr)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Map not yet loaded. Drivable area grid is unavailable.");
        return;
    }

    // printf("Publish grids... ");

    // Used to calculate function runtime
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    OccupancyGrid drivable_area_grid;
    OccupancyGrid junction_grid;
    OccupancyGrid route_dist_grid;
    MapMetaData grid_info;
    std::vector<int8_t> drivable_grid_data;
    std::vector<int8_t> junction_grid_data;
    std::vector<int8_t> route_dist_grid_data;

    float y_min = side_dist * -1;
    float y_max = side_dist;
    float x_min = bottom_dist * -1;
    float x_max = top_dist;

    // std::printf("[%f, %f], [%f, %f], %f\n", x_min, x_max, y_min, y_max, res);

    if (this->map_wide_tree_.size() == 0)
        this->map_wide_tree_ = this->map_->generate_mesh_tree();

    // Get the search region
    TransformStamped vehicle_tf = getVehicleTf();
    auto vehicle_pos = vehicle_tf.transform.translation;
    double range_plus = top_dist * 1.4; // This is a little leeway to account for map->base_link rotation
    odr::point bounding_box_min = odr::point(vehicle_pos.x - range_plus, vehicle_pos.y - range_plus);
    odr::point bounding_box_max = odr::point(vehicle_pos.x + range_plus, vehicle_pos.y + range_plus);
    odr::box search_region(bounding_box_min, bounding_box_max);

    // Find all lanes within the search region
    std::vector<odr::value> lane_shapes_in_range;
    map_wide_tree_.query(bgi::intersects(search_region), std::back_inserter(lane_shapes_in_range));

    // std::printf("There are %i shapes in range.\n", lane_shapes_in_range.size());

    int idx = 0;

    bgi::rtree<odr::value, bgi::rstar<16, 4>> local_tree;

    for (unsigned i = 0; i < lane_shapes_in_range.size(); ++i)
        local_tree.insert(lane_shapes_in_range.at(i));

    int area = 0;
    int height = 0;

    BoostPoint goal_pt;
    bool goal_is_set = false;
    auto q = vehicle_tf.transform.rotation;
    float h;

    if (q.z < 0)
        h = abs(2 * acos(q.w) - 2 * M_PI);
    else
        h = 2 * acos(q.w);

    if (h > M_PI)
        h -= 2 * M_PI;

    for (float j = y_min; j <= y_max; j += res)
    {
        for (float i = x_min; i <= x_max; i += res)
        {

            i = static_cast<float>(i);
            j = static_cast<float>(j);
            bool cell_is_drivable = false;
            bool cell_is_in_junction = false;

            // Transform this query into the map frame
            // First rotate, then translate. 2D rotation eq:
            // x' = xcos(h) - ysin(h)
            // y' = ycos(h) + xsin(h)

            // if (h < 0)
            //     h += 2 * M_PI;
            float i_in_map = i * cos(h) - j * sin(h) + vehicle_pos.x;
            float j_in_map = j * cos(h) + i * sin(h) + vehicle_pos.y;

            odr::point p(i_in_map, j_in_map);

            std::vector<odr::value> local_tree_query_results;
            local_tree.query(bgi::contains(p), std::back_inserter(local_tree_query_results));

            if (local_tree_query_results.size() > 0)
            {
                for (auto pair : local_tree_query_results)
                {
                    // auto pair = local_tree_query_results.front();
                    odr::ring ring = this->lane_polys_.at(pair.second).second;
                    odr::Lane lane = this->lane_polys_.at(pair.second).first;
                    // odr::Road road = this->map_->id_to_road.at(lane.key.road_id);
                    bool point_is_within_shape = bg::within(p, ring);
                    if (point_is_within_shape)
                    {

                        cell_is_in_junction = this->road_in_junction_map_[lane.key];
                        if (lane.type == "driving")
                        {
                            cell_is_drivable = true;
                            break;
                        }
                    }
                }
            }

            drivable_grid_data.push_back(cell_is_drivable ? 0 : 100);
            junction_grid_data.push_back(cell_is_in_junction ? 100 : 0);

            // Get closest route point
            if (local_route_linestring_.size() > 0 && cell_is_drivable && i > 0)
            {
                int dist = static_cast<int>(bg::distance(local_route_linestring_, p) * 8);

                if (dist < 1.0 && !goal_is_set && abs(i) + abs(j) > 30)
                {
                    goal_is_set = true;
                    goal_pt = BoostPoint(i, j);
                }

                // Distances > 10 are set to 100
                if (dist > 20)
                    dist = 100;
                else
                    dist *= 5;

                route_dist_grid_data.push_back(dist);
            }
            else
            {
                route_dist_grid_data.push_back(100);
            }

            area += 1;
        }
        height += 1;
    }

    auto clock = this->clock_->clock;
    drivable_area_grid.data = drivable_grid_data;
    drivable_area_grid.header.frame_id = "base_link";
    drivable_area_grid.header.stamp = clock;

    junction_grid.data = junction_grid_data;
    junction_grid.header.frame_id = "base_link";
    junction_grid.header.stamp = clock;

    route_dist_grid.data = route_dist_grid_data;
    route_dist_grid.header.frame_id = "base_link";
    route_dist_grid.header.stamp = clock;

    grid_info.width = area / height;
    grid_info.height = height;
    grid_info.map_load_time = clock;
    grid_info.resolution = res;
    grid_info.origin.position.x = x_min;
    grid_info.origin.position.y = y_min;

    // Set the grids' metadata
    drivable_area_grid.info = grid_info;
    junction_grid.info = grid_info;
    route_dist_grid.info = grid_info;

    // Output function runtime
    drivable_grid_pub_->publish(drivable_area_grid);
    junction_grid_pub_->publish(junction_grid);
    route_dist_grid_pub_->publish(route_dist_grid); // Route distance grid

    PoseStamped goal_pose;
    goal_pose.pose.position.x = goal_pt.get<0>();
    goal_pose.pose.position.y = goal_pt.get<1>();
    goal_pose.header.frame_id = "base_link";
    goal_pose.header.stamp = clock_->clock;
    goal_pose_pub_->publish(goal_pose);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "publishGrids(): " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;
}

/**
 * @brief Caches the latest clock from CARLA. Usand published for message timestamps.
 *
 * @param msg
 */
void MapManagementNode::clockCb(Clock::SharedPtr msg)
{
    this->clock_ = msg;
}

/**
 * @brief Gets latest map->base_link transform and returns it
 *
 * @return TransformStamped
 */
TransformStamped MapManagementNode::getVehicleTf()
{
    TransformStamped t;
    try
    {
        t = tf_buffer_->lookupTransform(
            "map", "base_link",
            tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Could not get base_link->map tf: %s. This will republish every 5 seconds.", ex.what());
        return TransformStamped();
    }
    return t;
}

/**
 * @brief Gets the drivable area OccupancyGrid and publishes it.
 *
 */
void MapManagementNode::drivableAreaGridPubTimerCb()
{
    // std::printf("Publishing grids\n");
    publishGrids(40, 20, 30, 0.4);
}

// CPP code for printing shortest path between
// two vertices of unweighted graph
#include <bits/stdc++.h>
using namespace std;

// utility function to form edge between two vertices
// source and dest
void add_edge(vector<int> adj[], int src, int dest)
{
    adj[src].push_back(dest);
    adj[dest].push_back(src);
}

/**
 * @brief Simple breadth-first search using libOpenDRIVE's inbuilt routing graph.
 * Undirected! See https://en.wikipedia.org/wiki/Breadth-first_search#Pseudocode
 *
 * @param source
 * @param dest
 * @param graph
 * @return std::unordered_map<odr::LaneKey, odr::LaneKey>
 */
std::unordered_map<odr::LaneKey, odr::LaneKey> bfs(odr::LaneKey source, odr::LaneKey dest, odr::RoutingGraph graph)
{
    std::unordered_set<odr::LaneKey> visited_lanes;
    std::unordered_map<odr::LaneKey, odr::LaneKey> parents;
    visited_lanes.insert(source);
    std::vector<odr::LaneKey> route;
    std::queue<odr::LaneKey> queue;

    queue.push(source);
    while (!queue.empty())
    {
        odr::LaneKey v = queue.front();
        queue.pop();

        if (v.to_string() == dest.to_string())
        {
            return parents;
        }

        for (auto w : graph.get_lane_predecessors(v))
        {
            if (visited_lanes.find(w) == visited_lanes.end())
            {
                visited_lanes.insert(w);
                parents.insert(std::make_pair(w, v));
                queue.push(w);
            }
        }
        for (auto w : graph.get_lane_successors(v))
        {
            if (visited_lanes.find(w) == visited_lanes.end())
            {
                visited_lanes.insert(w);
                parents.insert(std::make_pair(w, v));
                queue.push(w);
            }
        }
    }
    // RCLCPP_ERROR(get_logger(), "Destination not found");
    return parents;
}

LineString getSmoothSection(LineString full_route, BoostPoint ego, int &start_idx, int &end_idx)
{

    // Get closest waypoint in full_route
    float closest_dist = 999.9;
    int closest_idx = -1;

    for (int i = 0; i < full_route.size(); i++)
    {
        auto wp = full_route[i];
        float dist_to_ego = bg::distance(ego, wp);
        if (dist_to_ego < closest_dist)
        {
            closest_dist = dist_to_ego;
            closest_idx = i;
        }
    }

    end_idx = closest_idx < 1 ? closest_idx : closest_idx - 1;

    LineString rough_section;
    start_idx = closest_idx + 1;
    float edge_distance = closest_dist;

    // Find the "edge" waypoint
    // That's the divider between the smooth and rough route sections
    // It should be > 40m from the ego
    while (edge_distance < 40.0)
    {
        auto wp = full_route[start_idx];
        edge_distance = bg::distance(ego, wp);
        start_idx++;
    }

    return rough_section;
}

LineString MapManagementNode::getLaneCenterline(odr::LaneKey key)
{
    LineString centerline;

    odr::Road road = map_->id_to_road.at(key.road_id);
    odr::LaneSection lsec = road.s_to_lanesection.at(key.lanesection_s0);
    odr::Lane lane = lsec.id_to_lane.at(key.lane_id);

    odr::Line3D outer_border = road.get_lane_border_line(lane, 1.0, true);
    odr::Line3D inner_border = road.get_lane_border_line(lane, 1.0, false);

    for (int i = 0; i < outer_border.size(); i++)
    {
        odr::Vec3D outer_pt = outer_border[i];
        odr::Vec3D inner_pt = inner_border[i];
        BoostPoint center_pt((outer_pt[0] + inner_pt[0]) / 2, (outer_pt[1] + inner_pt[1]) / 2);
        bg::append(centerline, center_pt);
    }

    return centerline;
}

LineString getReorientedRoute(std::vector<LineString> centerlines)
{
    LineString result;

    auto iter = centerlines.begin();
    while (iter != centerlines.end() - 1)
    {
        // Get end of this segment and the beginning of the next one
        // Check the distance between them.
        // If distance is small, the orientation is correct. Continue.
        // If distance is large, reverse order of next segment

        const float MAX_ENDPOINT_GAP = 2.0; // meters

        auto current_end = iter->back();
        auto next_begin = (iter + 1)->front();
        float dist = bg::distance(current_end, next_begin);

        // std::printf("Distance was %f. ", dist);

        if (dist > MAX_ENDPOINT_GAP)
        {
            bg::reverse(*(iter + 1));
        }

        next_begin = (iter + 1)->front();
        // dist = bg::distance(current_end, next_begin);
        // std::printf("Distance is now %f.\n", dist);

        bg::append(result, *iter);

        iter++;
    }
    bg::append(result, centerlines.back());

    return result;
}

void MapManagementNode::updateRouteWaypoints(Path::SharedPtr msg)
{
    if (rough_route_tree_.size() > 0 && rough_route_tree_.size() == msg->poses.size())
    {
        // It looks like we've already processed a route and the new route as the same size.
        // If the new route's size and the current tree's size are the same,
        // we can assume that the incoming route is not new.
        return;
    }

    // Let's build an RTree to efficiently represent the new waypoints.

    rough_route_tree_.clear();
    rough_route_.clear();

    for (unsigned i = 0; i < msg->poses.size(); ++i)
    {
        PoseStamped wp_pose = msg->poses[i];

        BoostPoint wp(wp_pose.pose.position.x, wp_pose.pose.position.y);
        rough_route_tree_.insert(std::make_pair(bg::return_envelope<odr::box>(wp), i));
        bg::append(rough_route_, wp);
    }

    RCLCPP_INFO(get_logger(), "%i waypoints added to tree", rough_route_tree_.size());
}

RoiIndices getWaypointsInROI(LineString waypoints, bgi::rtree<odr::value, bgi::rstar<16, 4>> tree, BoostPoint ego_pos)
{
    std::vector<odr::value> returned_values;
    // Query our tree. "1" means get the single nearest waypoint
    tree.query(bgi::nearest(ego_pos, 1), std::back_inserter(returned_values));
    int nearest_idx = returned_values.front().second;



    // Go backward in rough route until either
    // a) We hit the route's start or
    // b) we're >40m from ego
    // Get idx of last waypoint visited
    auto iter = waypoints.begin() + nearest_idx;
    int start_idx = nearest_idx;
    while (iter != waypoints.begin())
    {
        start_idx--;
        iter--;
        if (bg::distance(ego_pos, *iter) > 40.0)
            break;
    }

    // Go forward in rough route until either
    // a) We hit the route's end or
    // b) we're >40m from ego
    // Get idx of last waypoint visited
    iter = waypoints.begin() + nearest_idx;
    int end_idx = nearest_idx;
    while (iter != waypoints.end())
    {
        end_idx++;
        iter++;
        if (bg::distance(ego_pos, *iter) > 40.0)
            break;
    }

    // RCLCPP_INFO(get_logger(), "(%i, %i, %i)", start_idx, nearest_idx, end_idx);

    RoiIndices result;
    result.center = nearest_idx;
    result.start = start_idx;
    result.end = end_idx;

    return result;
}

std::vector<odr::LaneKey> getLaneKeysFromIndices(RoiIndices indices, LineString waypoints, bgi::rtree<odr::value, bgi::rstar<16, 4>> lane_tree, std::vector<odr::LanePair> lane_polys)
{
    LineString waypoints_within_roi;
    // bg::simplify(waypoints, simplified_wps, 5.0);

    for (int i = indices.start; i <= indices.end; i++)
    {
        bg::append(waypoints_within_roi, waypoints[i]);
    }

    // Simplification downsamples the linestring, speeding us up a bit
    LineString simplified_wpts;
    bg::simplify(waypoints_within_roi, simplified_wpts, 5.0);

    std::vector<odr::value> query_results;

    // For each wp, find it in the lane_tree
    for (auto pt : simplified_wpts)
    {
        lane_tree.query(bgi::intersects(pt), std::back_inserter(query_results));
    }

    std::vector<odr::LaneKey> lane_keys;

    // Move through our tree search results, extracting the lane key and
    // adding it to our result list.
    for (auto result : query_results)
    {
        odr::LanePair result_pair = lane_polys[result.second];
        lane_keys.push_back(result_pair.first.key);
        // std::printf("%s\n", result_pair.first.key.to_string().c_str());
    }

    return lane_keys;
}

std::vector<LineString> MapManagementNode::getCenterlinesFromKeys(std::vector<odr::LaneKey> keys, odr::RoutingGraph graph)
{

    // Move from first to second-to-last key.
    // Use VFS to find route from this key to the next one,
    // collectively forming a route chain connecting each waypoint's parent lane

    std::vector<odr::LaneKey> complete_keys; // keys, but with gaps in lanes filled.

    // Loop starts at last key and works to the first key
    for (auto iter = keys.end() - 1; iter != keys.begin(); iter--)
    {
        std::vector<odr::LaneKey> complete_segment; // keys, but with gaps in lanes filled.
        odr::LaneKey from = *iter;
        odr::LaneKey to = *(iter - 1);
        // std::printf("(%s)=>(%s)\n", from.to_string().c_str(), to.to_string().c_str());
        if (from.to_string() == to.to_string()) // Keys have the same lane
            continue;
        auto adjacency_pairs = bfs(from, to, graph);
        // std::printf("BFS returned %i pairs\n", adjacency_pairs.size());

        complete_segment.push_back(to);

        // Work our way back from destination to source in the tree search,
        // producing a continuous LaneKey route.
        try
        {
            odr::LaneKey parent = adjacency_pairs.at(to);
            do
            {
                complete_segment.push_back(parent);
                parent = adjacency_pairs.at(parent);
            } while (parent.to_string() != from.to_string());
        }
        catch (...)
        {
            // RCLCPP_ERROR(get_logger(), "BFS could not locate route parent. Returning.");
        }

        complete_keys.insert(complete_keys.begin(), complete_segment.begin(), complete_segment.end());
    }

    // The final key is left out of above. Add it now.
    complete_keys.push_back(keys.back());

    // We now have a continuous LaneKey sequence that connects all provided Keys.
    std::vector<LineString> centerlines;
    for (auto key : complete_keys)
    {
        // std::printf("%s\n", key.to_string().c_str());

        LineString centerline = getLaneCenterline(key);
        centerlines.push_back(centerline);
    }

    return centerlines;
}

/**
 * @brief Given rough waypoints, turn them into a smooth, local curve
 * of continuous lane centerlines.
 *
 * 1. Crop waypoints to those nearby (<40m, plus 1-waypoint buffer on either end)
 * 2. Convert waypoints to matching lanes (odr::LaneKeys)
 * 3. Convert LaneKeys to LineStrings from BoostGeometry
 * 4. Reorient these LineStrings so that their ends line up. Combine them into a single curve.
 * 5. Convert the final LineString to a path message.
 *
 */
void MapManagementNode::publishRefinedRoute()
{
    if (rough_route_tree_.empty() || map_wide_tree_.empty())
        return;

    // Get waypoint closest to ego
    auto ego_tf = getVehicleTf();
    BoostPoint ego_pos(ego_tf.transform.translation.x, ego_tf.transform.translation.y);

    // Get indices of waypoint ROI
    RoiIndices waypoint_roi = getWaypointsInROI(rough_route_, rough_route_tree_, ego_pos);

    // Let's take a moment to publish our route progress using this info
    float route_progress = static_cast<float>(waypoint_roi.center) / rough_route_.size();
    std_msgs::msg::Float32 progress_msg;
    progress_msg.data = route_progress;
    route_progress_pub_->publish(progress_msg);

    // Get LaneKeys
    std::vector<odr::LaneKey> keys = getLaneKeysFromIndices(waypoint_roi, rough_route_, map_wide_tree_, lane_polys_);

    // Get centerlines
    auto centerlines = getCenterlinesFromKeys(keys, map_->get_routing_graph());

    // Orient them so that their ends and beginnings properly match
    LineString route_ls = getReorientedRoute(centerlines);
    bg::simplify(route_ls, local_route_linestring_, 1.0);

    // Convert to ROS message
    Path result;
    result.header.frame_id = "map";
    result.header.stamp = clock_->clock;
    for (auto pt : route_ls)
    {
        PoseStamped pose;
        pose.pose.position.x = pt.get<0>();
        pose.pose.position.y = pt.get<1>();
        pose.header.frame_id = "map";
        pose.header.stamp = clock_->clock;
        result.poses.push_back(pose);
    }

    route_path_pub_->publish(result);
}

std::map<odr::LaneKey, bool> MapManagementNode::getJunctionMap(std::vector<odr::LanePair> lane_polys)
{
    std::map<odr::LaneKey, bool> map;
    for (auto pair : lane_polys)
    {
        odr::Lane lane = pair.first;
        odr::Road road = this->map_->id_to_road.at(lane.key.road_id);
        if (road.junction != "-1" && lane.type == "driving")
            map[lane.key] = true;
        else
            map[lane.key] = false;
    }

    return map;
}

/**
 * @brief When map data is received from a CarlaWorldInfo message, load and process it
 *
 * @param msg The incoming CarlaWorldInfo message
 */
void MapManagementNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
    if (this->map_ != nullptr)
        return; // Our map is already loaded. No need to continue.
    if (msg->opendrive == "")
    {
        RCLCPP_INFO(this->get_logger(), "Received empty map string from world_info topic. Waiting for real data.");
        return;
    }

    // Ask libopendrive to parse the map string
    this->map_ = new odr::OpenDriveMap(msg->opendrive, true);
    // Get lane polygons as pairs (Lane object, ring polygon)
    this->lane_polys_ = map_->get_lane_polygons(1.0, false);

    this->road_in_junction_map_ = this->getJunctionMap(this->lane_polys_);

    RCLCPP_INFO(this->get_logger(), "Loaded %s", msg->map_name.c_str());
}