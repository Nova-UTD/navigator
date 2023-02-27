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
    route_path_pub_ = this->create_publisher<Path>("/planning/smooth_route", 10);
    traffic_light_points_pub_ = this->create_publisher<PolygonStamped>("/traffic_light_points", 10);
    goal_pose_pub_ = this->create_publisher<PoseStamped>("/planning/goal_pose", 1);

    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    rough_path_sub_ = this->create_subscription<Path>("/planning/rough_route", 10, bind(&MapManagementNode::refineRoughRoute, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));

    drivable_area_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::drivableAreaGridPubTimerCb, this));
    // route_distance_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::updateRoute, this));
    route_timer_ = this->create_wall_timer(ROUTE_PUBLISH_FREQUENCY, bind(&MapManagementNode::updateRoute, this));
    // traffic_light_pub_timer_ = this->create_wall_timer(TRAFFIC_LIGHT_PUBLISH_FREQUENCY, [this]()
    //                                                    { this->traffic_light_points_pub_->publish(traffic_light_points); });

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
    // std::printf("Publishing grids\n");
    publishGrids(40, 20, 30, 0.4);
}


LineString getRoughSection(LineString full_route, BoostPoint ego)
{
    float MIN_START_DISTANCE = 40.0; // meters. Start of rough section must be at least this far away.

    LineString rough_section;

    for (auto pt : full_route)
    {
        if(bg::distance(ego, pt) < MIN_START_DISTANCE)
            rough_section.clear();
        else
            bg::append(rough_section, pt);
    }

    return rough_section;
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
    printf("Refine rough route...\n");
    TransformStamped egoTf = getVehicleTf();
    BoostPoint ego_pos(egoTf.transform.translation.x, egoTf.transform.translation.y);

    LineString full_route;

    for (auto pose : msg->poses)
    {
        BoostPoint route_pt(pose.pose.position.x, pose.pose.position.y);
        bg::append(full_route, route_pt);
    }

    LineString rough_section = getRoughSection(full_route, ego_pos);

    // Publish result as a Path message
    Path result;
    result.header.stamp = clock_->clock;
    result.header.frame_id = "map";

    for (auto pt : rough_section)
    {
        PoseStamped waypoint_pose;
        waypoint_pose.pose.position.x = pt.get<0>();
        waypoint_pose.pose.position.y = pt.get<1>();
        waypoint_pose.header.stamp = clock_->clock;
        waypoint_pose.header.frame_id = "map";
        result.poses.push_back(waypoint_pose);
    }

    route_path_pub_->publish(result);
}

/**
 * @brief Generate and publish the "distance from route" cost map layer
 *
 */
void MapManagementNode::updateRoute()
{
    // printf("updateRoute() 565\n");

    // TransformStamped vehicle_tf = getVehicleTf();
    // printf("updateRoute() 568\n");

    // auto pos_msg = vehicle_tf.transform.translation;
    // BoostPoint vehicle_pos(pos_msg.x, pos_msg.y);

    // LineString route = rm.getRoute(rough_route_, vehicle_pos);

    // if (route_tree_.size() < 1)
    //     return;

    // // Get all points in front of car
    // // Transform this query into the map frame
    // // First rotate, then translate. 2D rotation eq:
    // // x' = xcos(h) - ysin(h)
    // // y' = ycos(h) + xsin(h)
    // double range_plus = GRID_RANGE * 1.4; // This is a little leeway to account for map->base_link rotation
    // odr::point bounding_box_min = odr::point(vehicle_pos.get<0>() - range_plus, vehicle_pos.get<1>() - range_plus);
    // odr::point bounding_box_max = odr::point(vehicle_pos.get<0>() + range_plus, vehicle_pos.get<1>() + range_plus);
    // odr::box search_region(bounding_box_min, bounding_box_max);

    // // Find all route points within the search region
    // std::vector<odr::value> nearest_route_pt_vector;
    // odr::point current_pos = odr::point(vehicle_pos.get<0>(), vehicle_pos.get<1>());
    // route_tree_.query(bgi::nearest(current_pos, 1), std::back_inserter(nearest_route_pt_vector));

    // local_route_linestring_.clear();

    // int nearest_route_pt_idx = nearest_route_pt_vector[0].second;

    // int length = 10; // Number of route points to include after the nearest one

    // for (int i = 0; i < length; i++)
    // {
    //     bg::append(local_route_linestring_, route_linestring_[nearest_route_pt_idx - i]);
    // }

    // RCLCPP_INFO(get_logger(), "Publishing smoothed route");
    // route_path_pub_->publish(this->rough_route_msg_);
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

    RCLCPP_INFO(this->get_logger(), "Loaded %s", msg->map_name.c_str());
}