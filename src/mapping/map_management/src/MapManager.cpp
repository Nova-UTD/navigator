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

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    // Publishers and subscribers
    drivable_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/drivable", 10);
    flat_surface_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/flat_surface", 10);
    route_dist_grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/route_distance", 10);
    route_path_pub_ = this->create_publisher<Path>("/route/smooth_path", 10);
    traffic_light_points_pub_ = this->create_publisher<PolygonStamped>("/traffic_light_points", 10);
    goal_pose_pub_ = this->create_publisher<PoseStamped>("/planning/goal_pose", 1);

    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    rough_path_sub_ = this->create_subscription<Path>("/route/rough_path", 10, bind(&MapManagementNode::refineRoughRoute, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));

    drivable_area_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::drivableAreaGridPubTimerCb, this));
    // route_distance_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::updateRoute, this));
    route_timer_ = this->create_wall_timer(ROUTE_PUBLISH_FREQUENCY, bind(&MapManagementNode::updateRoute, this));
    traffic_light_pub_timer_ = this->create_wall_timer(TRAFFIC_LIGHT_PUBLISH_FREQUENCY, [this]()
                                                       { this->traffic_light_points_pub_->publish(traffic_light_points); });

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

    // Used to calculate function runtime
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    OccupancyGrid drivable_area_grid;
    OccupancyGrid flat_surface_grid;
    OccupancyGrid route_dist_grid;
    MapMetaData grid_info;
    std::vector<int8_t> drivable_grid_data;
    std::vector<int8_t> flat_surface_grid_data;
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

    bool goalPoseIsPublished = false;

    for (float j = y_min; j <= y_max; j += res)
    {
        for (float i = x_min; i <= x_max; i += res)
        {

            i = static_cast<float>(i);
            j = static_cast<float>(j);
            bool cell_is_drivable = false;
            bool cell_is_flat_surface = false;

            // Transform this query into the map frame
            // First rotate, then translate. 2D rotation eq:
            // x' = xcos(h) - ysin(h)
            // y' = ycos(h) + xsin(h)
            float h = 2 * asin(vehicle_tf.transform.rotation.z); // TODO: Do not assume flat ground!
            float i_in_map = i * cos(h) - j * sin(h) + vehicle_pos.x;
            float j_in_map = j * cos(h) + i * sin(h) + vehicle_pos.y;

            odr::point p(i_in_map, j_in_map);

            std::vector<odr::value> local_tree_query_results;
            local_tree.query(bgi::contains(p), std::back_inserter(local_tree_query_results));

            if (local_tree_query_results.size() > 0)
            {
                for (auto pair : local_tree_query_results)
                {
                    odr::ring ring = this->lane_polys_.at(pair.second).second;
                    odr::Lane lane = this->lane_polys_.at(pair.second).first;
                    bool point_is_within_shape = bg::within(p, ring);
                    if (point_is_within_shape)
                    {
                        if (lane.type == "driving")
                        {
                            cell_is_drivable = true;
                            cell_is_flat_surface = true;
                            break;
                        }
                        else if (lane.type == "shoulder" || lane.type == "sidewalk" || lane.type == "parking" || lane.type == "curb")
                        {
                            cell_is_flat_surface = true;
                            break;
                        }
                    }
                }
            }

            drivable_grid_data.push_back(cell_is_drivable ? 0 : 100);
            flat_surface_grid_data.push_back(cell_is_flat_surface ? 0 : 100);

            // Get closest route point
            if (local_route_linestring_.size() > 0)
            {
                int dist = static_cast<int>(bg::distance(local_route_linestring_, p) * 2);

                // Distances > 10 are set to 100
                if (dist > 20)
                    dist = 100;
                else
                    dist *= 5;

                if (!goalPoseIsPublished && dist < 0.8)
                {
                    odr::point origin = odr::point(0.0, 0.0);
                    float distToCar = bg::distance(p, origin);
                    if (distToCar > 20)
                    {
                        PoseStamped goal_pose;
                        goal_pose.header.stamp = this->clock_->clock;
                        goal_pose.header.frame_id = "base_link";
                        goal_pose.pose.position.x = i;
                        goal_pose.pose.position.y = j;
                        goal_pose_pub_->publish(goal_pose);
                        goalPoseIsPublished = true;
                    }
                }

                route_dist_grid_data.push_back(dist);
            }
            else
            {
                route_dist_grid_data.push_back(-1);
            }

            area += 1;
        }
        height += 1;
    }

    auto clock = this->clock_->clock;
    drivable_area_grid.data = drivable_grid_data;
    drivable_area_grid.header.frame_id = "base_link";
    drivable_area_grid.header.stamp = clock;

    flat_surface_grid.data = flat_surface_grid_data;
    flat_surface_grid.header.frame_id = "base_link";
    flat_surface_grid.header.stamp = clock;

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
    flat_surface_grid.info = grid_info;
    route_dist_grid.info = grid_info;

    // Output function runtime
    drivable_grid_pub_->publish(drivable_area_grid);
    flat_surface_grid_pub_->publish(flat_surface_grid);
    route_dist_grid_pub_->publish(route_dist_grid); // Route distance grid

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
    publishGrids(40, 20, 30, 0.4);
}

/**
 * @brief Refine a rough path from CARLA into a smooth, sub-meter accurate path
 *
 * 1. Receive rough route from CARLA
 * 2. Get closest route point to car
 * 3. Trim all points not within interval (closestPoint, goalPoint), where the goal point is the final point in the list
 * 4. While distanceFromCar < some param, insert refined points. using HD map data for lane centerlines
 *
 * @param msg
 */
void MapManagementNode::refineRoughRoute(Path::SharedPtr msg)
{
    if (this->map_ == nullptr)
        return;

    // Get ego position
    TransformStamped egoTf = getVehicleTf();

    rough_route_.clear();
    for (auto pose : msg->poses)
    {
        BoostPoint pt(pose.pose.position.x, pose.pose.position.y);
        bg::append(rough_route_, pt);
    }

    // // Get closest route point

    // float min_distance = 999.9;
    // size_t closest_idx = 999; // Yes, this is jank.

    // for (size_t i = 0; i < msg->poses.size(); i++)
    // {
    //     PoseStamped pose = msg->poses[i];
    //     PointMsg point = pose.pose.position;
    //     auto currentPos = egoTf.transform.translation;
    //     // Euclidean distance
    //     float dist = sqrt(pow(point.x - currentPos.x, 2) + (point.y - currentPos.y, 2));

    //     if (dist < min_distance)
    //     {
    //         min_distance = dist;
    //         closest_idx = i;
    //     }
    // }
    // RCLCPP_INFO(this->get_logger(), "Closest pose was at %i with dist %f", closest_idx, min_distance);
    // std::vector<odr::Lane> lanes_in_route;

    // RCLCPP_INFO(this->get_logger(), "Processing rough path with %i poses", msg->poses.size());

    // for (auto pose : msg->poses)
    // {
    //     auto position = pose.pose.position;
    //     // Find the lane polygon(s) that the point falls within
    //     odr::point p(position.x, position.y);
    //     std::vector<odr::value> query_results;
    //     std::vector<odr::Lane> lanes_containing_point;

    //     this->map_wide_tree_.query(bgi::contains(p), std::back_inserter(query_results));

    //     // RCLCPP_INFO(this->get_logger(), "Query returned %i results", query_results.size());

    //     for (auto result : query_results)
    //     {
    //         /*
    //         lane_polys_ is a vector of LanePairs, where a LanePair
    //         has a Lane object (which contains details like the lane ID, parent road, etc),
    //         and a ring (a Boost geometry that describes the outline of the lane).

    //         A "result" contains a given lane ring's bounding box (first) and
    //         the index of the ring (result.second). This is the same index within
    //         lane_polys_. Adding ".first" gets the Lane object.

    //         Got that?
    //         */

    //         odr::ring ring = this->lane_polys_.at(result.second).second;
    //         if (bg::within(p, ring))
    //         {
    //             odr::Lane containing_lane = this->lane_polys_.at(result.second).first;

    //             auto parent_road = this->map_->id_to_road.at(containing_lane.key.road_id);
    //             if (parent_road.junction != "-1")
    //                 continue; // Junction found! Skip this lane.

    //             lanes_containing_point.push_back(containing_lane);
    //         }
    //     }

    //     // Does this point fall within more than one lane?
    //     // If so, we've found a junction. Skip to next pose.
    //     // if (lanes_containing_point.size() > 1)
    //     // {
    //     //     RCLCPP_INFO(this->get_logger(), "Point (%f, %f) falls within a junction. Skipping.", position.x, position.y);
    //     //     continue;
    //     // }
    //     if (lanes_containing_point.size() < 1)
    //     {
    //         RCLCPP_INFO(this->get_logger(), "Point (%f, %f) does not fall within a lane. Skipping.", position.x, position.y);
    //         continue;
    //     }

    //     // Append the matching lane to the full route
    //     lanes_in_route.push_back(lanes_containing_point.front());
    // }

    // this->lanes_in_route_ = lanes_in_route;

    // smoothed_route_msg_.poses.clear();
    // smoothed_route_msg_.header.frame_id = "map";
    // smoothed_route_msg_.header.stamp = this->clock_->clock;

    // auto routing_graph = this->map_->get_routing_graph();

    // std::vector<odr::LaneKey> complete_route;

    // int progress_counter = 0;
    // const int ITER_LIMIT = 10;

    // for (auto it = lanes_in_route.begin(); it < lanes_in_route.end() - 1; it++)
    // {
    //     printf("%i/%i\n", progress_counter, lanes_in_route.size());
    //     progress_counter++;

    //     if (progress_counter > ITER_LIMIT)
    //         break;

    //     if (it->key.to_string() == (it + 1)->key.to_string())
    //     {
    //         // printf("Keys were the same. Skipping.\n");
    //         continue;
    //     }
    //     std::vector<odr::LaneKey> route = routing_graph.shortest_path((it + 1)->key, it->key);

    //     // Reverse the order of the route
    //     std::reverse(route.begin(), route.end());

    //     if (route.size() < 2)
    //     {
    //         // RCLCPP_WARN(this->get_logger(), "Route not found. Trying other direction.");
    //         route = routing_graph.shortest_path(it->key, (it + 1)->key);
    //         if (route.size() < 2)
    //         {
    //             RCLCPP_WARN(this->get_logger(), "Route not found.");
    //             continue;
    //         }
    //     }
    //     for (auto key : route)
    //     {
    //         if (complete_route.size() > 0 && key.to_string() == complete_route.back().to_string())
    //         {
    //             continue;
    //         }

    //         complete_route.push_back(key);
    //     }

    //     if (progress_counter % 10 == 0)
    //     {
    //         for (auto key : complete_route)
    //         {
    //             printf("%s\n", key.to_string().c_str());
    //         }
    //     }
    // }

    // // For each LaneKey in Route, linear sample and append to vector
    // bg::model::linestring<odr::point> route_linestring;

    // odr::point latest_point;

    // for (auto key : complete_route)
    // {
    //     // printf("%s\n", key.to_string().c_str());
    //     odr::Road road = this->map_->id_to_road.at(key.road_id);
    //     // printf("Road was %s\n", road.id.c_str());
    //     odr::LaneSection lsec = road.get_lanesection(key.lanesection_s0);
    //     // printf("Lsec was %f\n", lsec.s0);
    //     // for (auto lane : lsec.get_lanes())
    //     // {
    //     //     printf("%i ", lane.id);
    //     // }
    //     printf("\n");
    //     odr::Lane lane = lsec.id_to_lane.at(key.lane_id);
    //     // printf("Lane was %i\n", lane.id);

    //     odr::Line3D outer_border = road.get_lane_border_line(lane, 1.0, true);
    //     odr::Line3D inner_border = road.get_lane_border_line(lane, 1.0, false);

    //     odr::Line3D line;

    //     for (int i = 0; i < outer_border.size(); i++)
    //     {
    //         odr::Vec3D center_pt;
    //         auto inner_pt = inner_border[i];
    //         auto outer_pt = outer_border[i];
    //         // RCLCPP_INFO(get_logger(), "Inner: (%f, %f)\n", inner_pt[0], inner_pt[1]);
    //         // RCLCPP_INFO(get_logger(), "Outer: (%f, %f)\n", outer_pt[0], outer_pt[1]);

    //         center_pt[0] = (inner_pt[0] + outer_pt[0]) / 2;
    //         center_pt[1] = (inner_pt[1] + outer_pt[1]) / 2;
    //         line.push_back(center_pt);
    //     }

    //     // Test to see if the direction of this lane needs to be reversed
    //     if (route_linestring.size() > 0)
    //     {

    //         odr::point first_pt_in_lane = odr::point(line.front()[0], line.front()[1]);
    //         double dist = bg::distance(latest_point, first_pt_in_lane);
    //         printf("Distance was %f\n", dist);

    //         if (dist > 30)
    //         {
    //             std::reverse(line.begin(), line.end());
    //             odr::point first_pt_in_lane = odr::point(line.front()[0], line.front()[1]);
    //             double dist = bg::distance(latest_point, first_pt_in_lane);
    //             printf("Distance is now %f\n", dist);
    //         }
    //     }
    //     else
    //     {
    //         latest_point = odr::point(line.front()[0], line.front()[1]);
    //     }

    //     for (odr::Vec3D pt : line)
    //     {
    //         odr::point boost_point(pt[0], pt[1]);
    //         bg::append(route_linestring, boost_point);
    //         printf("Adding point (%f, %f)\n", pt[0], pt[1]);
    //         latest_point = boost_point;
    //     }
    // }

    // // Publish route as Path msg
    // smoothed_route_msg_.header.stamp = this->clock_->clock;
    // smoothed_route_msg_.header.frame_id = "map";

    // smoothed_route_msg_.poses.clear();
    // for (odr::point pt : route_linestring)
    // {
    //     PoseStamped pose_msg;
    //     pose_msg.pose.position.x = pt.get<0>();
    //     pose_msg.pose.position.y = pt.get<1>();
    //     pose_msg.header = smoothed_route_msg_.header;
    //     smoothed_route_msg_.poses.push_back(pose_msg);
    // }

    // route_path_pub_->publish(smoothed_route_msg_);
    // this->route_linestring_ = route_linestring;

    // bgi::rtree<odr::value, bgi::rstar<16, 4>> rtree;

    // // Build the route rtree
    // for (unsigned i = 0; i < route_linestring.size(); ++i)
    // {
    //     // calculate polygon bounding box
    //     odr::box b = bg::return_envelope<odr::box>(route_linestring[i]);
    //     // insert new value
    //     rtree.insert(std::make_pair(b, i));
    //     // std::printf("Inserting box (%f, %f)-(%f,%f)\n", b.min_corner().get<0>(), b.min_corner().get<1>(), b.max_corner().get<0>(), b.max_corner().get<1>());
    // }

    // this->route_tree_ = rtree;

    // RCLCPP_INFO(this->get_logger(), "Publishing path with %i poses. Started with %i poses.\n", smoothed_route_msg_.poses.size(), msg->poses.size());
}

/**
 * @brief Generate and publish the "distance from route" cost map layer
 *
 */
void MapManagementNode::updateRoute()
{
    TransformStamped vehicle_tf = getVehicleTf();
    auto pos_msg = vehicle_tf.transform.translation;
    BoostPoint vehicle_pos(pos_msg.x, pos_msg.y);

    LineString route = rm.getRoute(rough_route_, vehicle_pos);

    if (route_tree_.size() < 1)
        return;

    // Get all points in front of car
    // Transform this query into the map frame
    // First rotate, then translate. 2D rotation eq:
    // x' = xcos(h) - ysin(h)
    // y' = ycos(h) + xsin(h)
    double range_plus = GRID_RANGE * 1.4; // This is a little leeway to account for map->base_link rotation
    odr::point bounding_box_min = odr::point(vehicle_pos.get<0>() - range_plus, vehicle_pos.get<1>() - range_plus);
    odr::point bounding_box_max = odr::point(vehicle_pos.get<0>() + range_plus, vehicle_pos.get<1>() + range_plus);
    odr::box search_region(bounding_box_min, bounding_box_max);

    // Find all route points within the search region
    std::vector<odr::value> nearest_route_pt_vector;
    odr::point current_pos = odr::point(vehicle_pos.get<0>(), vehicle_pos.get<1>());
    route_tree_.query(bgi::nearest(current_pos, 1), std::back_inserter(nearest_route_pt_vector));

    local_route_linestring_.clear();

    int nearest_route_pt_idx = nearest_route_pt_vector[0].second;

    int length = 10; // Number of route points to include after the nearest one

    for (int i = 0; i < length; i++)
    {
        bg::append(local_route_linestring_, route_linestring_[nearest_route_pt_idx - i]);
    }

    RCLCPP_INFO(get_logger(), "Publishing smoothed route");
    route_path_pub_->publish(this->smoothed_route_msg_);
}

PolygonStamped MapManagementNode::getTrafficLightCloud(std::vector<std::pair<odr::RoadObject, odr::point>> object_centers)
{
    int qty = 0;
    PolygonStamped polygon;
    polygon.header.stamp = this->clock_->clock;
    polygon.header.frame_id = "map";
    for (auto pair : object_centers)
    {
        odr::RoadObject obj = pair.first;
        odr::point center = pair.second;
        if (obj.name != "Signal_3Light_Post01")
            continue;
        qty++;
        Point32 point;
        point.x = center.get<0>();
        point.y = center.get<1>();
        point.z = 0;
        polygon.polygon.points.push_back(point);
    }
    RCLCPP_INFO(this->get_logger(), "%i traffic lights", qty);
    return polygon;
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
    // Get road objects
    this->traffic_light_points = getTrafficLightCloud(map_->get_road_object_centers());

    RCLCPP_INFO(this->get_logger(), "Loaded %s", msg->map_name.c_str());
}