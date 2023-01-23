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

using namespace navigator::perception;

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    // Publishers and subscribers
    grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/drivable", 10);
    route_path_pub_ = this->create_publisher<Path>("/route/smooth_path", 10);

    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    rough_path_sub_ = this->create_subscription<Path>("/route/rough_path", 10, bind(&MapManagementNode::refineRoughPath, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));

    drivable_area_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::drivableAreaGridPubTimerCb, this));
    route_distance_grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::routeDistanceGridPubTimerCb, this));

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
 * @param range Radius in meters
 * @param res Side length of grid cells (meters)
 * @return OccupancyGrid
 */
OccupancyGrid navigator::perception::MapManagementNode::getDrivableAreaGrid(PointMsg center, int range, float res)
{
    if (this->map_ == nullptr)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Map not yet loaded. Drivable area grid is unavailable.");
        return OccupancyGrid();
    }

    // Used to calculate function runtime
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    OccupancyGrid occupancy_grid;
    std::vector<int8_t> grid_data;

    int grid_radius_in_cells = std::ceil(range / res);

    float x_min = center.x - range;
    float x_max = center.x + range;
    float y_min = center.y - range;
    float y_max = center.y + range;

    // std::printf("[%f, %f], [%f, %f], %f\n", x_min, x_max, y_min, y_max, res);

    if (this->map_wide_tree_.size() == 0)
        this->map_wide_tree_ = this->map_->generate_mesh_tree();

    odr::box search_region(odr::point(x_min, y_min), odr::point(x_max, y_max));
    std::vector<odr::value> lane_shapes_in_range;
    map_wide_tree_.query(bgi::intersects(search_region), std::back_inserter(lane_shapes_in_range));

    // std::printf("There are %i shapes in range.\n", lane_shapes_in_range.size());

    int idx = 0;

    bgi::rtree<odr::value, bgi::rstar<16, 4>> local_tree;

    for (unsigned i = 0; i < lane_shapes_in_range.size(); ++i)
        local_tree.insert(lane_shapes_in_range.at(i));

    int area = 0;
    int height = 0;

    for (float j = y_min; j <= y_max; j += res)
    {
        for (float i = x_min; i <= x_max; i += res)
        {

            i = static_cast<float>(i);
            j = static_cast<float>(j);
            bool cell_is_occupied = false;

            odr::point p(i, j);

            std::vector<odr::value> local_tree_query_results;
            local_tree.query(bgi::contains(p), std::back_inserter(local_tree_query_results));

            if (local_tree_query_results.size() > 0)
            {
                for (auto pair : local_tree_query_results)
                {
                    odr::ring ring = this->lane_polys_.at(pair.second).second;
                    bool point_is_within_shape = bg::within(p, ring);
                    if (point_is_within_shape)
                    {
                        cell_is_occupied = true;

                        break;
                    }
                }
            }

            if (cell_is_occupied)
                grid_data.push_back(100);
            else
                grid_data.push_back(0);

            area += 1;
        }
        height += 1;
    }

    // Set the OccupancyGrid's metadata
    auto clock = this->clock_->clock;
    occupancy_grid.data = grid_data;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = clock;
    occupancy_grid.info.width = area / height;
    occupancy_grid.info.height = height;
    occupancy_grid.info.map_load_time = clock;
    occupancy_grid.info.resolution = GRID_RES;
    occupancy_grid.info.origin.position.x = x_min;
    occupancy_grid.info.origin.position.y = y_min;

    // Output function runtime
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    // std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return occupancy_grid;
}

/**
 * @brief Caches the latest clock from CARLA. Usand published for message timestamps.
 *
 * @param msg
 */
void navigator::perception::MapManagementNode::clockCb(Clock::SharedPtr msg)
{
    this->clock_ = msg;
}

/**
 * @brief Gets latest map->base_link transform and returns it
 *
 * @return TransformStamped
 */
TransformStamped navigator::perception::MapManagementNode::getVehicleTf()
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
void navigator::perception::MapManagementNode::drivableAreaGridPubTimerCb()
{
    TransformStamped t = getVehicleTf();

    PointMsg center;
    center.x = t.transform.translation.x;
    center.y = t.transform.translation.y;

    OccupancyGrid msg = getDrivableAreaGrid(center, GRID_RANGE, GRID_RES);
    grid_pub_->publish(msg);
}

/**
 * @brief Refine a rough path from CARLA into a smooth, sub-meter accurate path
 * 
 * 1. For each pose in the rough path, use an RTree to find the matching lane in the map
 * 2. Given the sequence of lanes, fill in gaps and validate using libOpenDRIVE's routing graph
 *  a. This is very, very slow, but it should guarantee a valid lane sequence.
 *  b. This accounts for junctions, where a point may belong to >1 lane.
 * 3. For each lane in the sequence, sample the centerline and add this point to a boost linestring
 *  a. This creates a continuous Cartesian curve
 * 4. Pusblish this refined curve as a Path message.
 * 
 * @param msg 
 */
void navigator::perception::MapManagementNode::refineRoughPath(Path::SharedPtr msg)
{
    if (this->map_ == nullptr)
        return;
    if (this->smoothed_path_msg_.poses.size() > 0)
    {
        // RCLCPP_WARN(this->get_logger(), "Lanes already calculated from rough path.");
        // route_path_pub_->publish(smoothed_path_msg_);
        return;
    }

    std::vector<odr::Lane> lanes_in_route;

    RCLCPP_INFO(this->get_logger(), "Processing rough path with %i poses", msg->poses.size());

    for (auto pose : msg->poses)
    {
        auto position = pose.pose.position;
        // Find the lane polygon(s) that the point falls within
        odr::point p(position.x, position.y);
        std::vector<odr::value> query_results;
        std::vector<odr::Lane> lanes_containing_point;

        this->map_wide_tree_.query(bgi::contains(p), std::back_inserter(query_results));

        // RCLCPP_INFO(this->get_logger(), "Query returned %i results", query_results.size());

        for (auto result : query_results)
        {
            /*
            lane_polys_ is a vector of LanePairs, where a LanePair
            has a Lane object (which contains details like the lane ID, parent road, etc),
            and a ring (a Boost geometry that describes the outline of the lane).

            A "result" contains a given lane ring's bounding box (first) and
            the index of the ring (result.second). This is the same index within
            lane_polys_. Adding ".first" gets the Lane object.

            Got that?
            */

            odr::ring ring = this->lane_polys_.at(result.second).second;
            if (bg::within(p, ring))
            {
                odr::Lane containing_lane = this->lane_polys_.at(result.second).first;

                auto parent_road = this->map_->id_to_road.at(containing_lane.key.road_id);
                if (parent_road.junction != "-1")
                    continue; // Junction found! Skip this lane.

                lanes_containing_point.push_back(containing_lane);
            }
        }

        // Does this point fall within more than one lane?
        // If so, we've found a junction. Skip to next pose.
        // if (lanes_containing_point.size() > 1)
        // {
        //     RCLCPP_INFO(this->get_logger(), "Point (%f, %f) falls within a junction. Skipping.", position.x, position.y);
        //     continue;
        // }
        if (lanes_containing_point.size() < 1)
        {
            RCLCPP_INFO(this->get_logger(), "Point (%f, %f) does not fall within a lane. Skipping.", position.x, position.y);
            continue;
        }

        // Append the matching lane to the full route
        lanes_in_route.push_back(lanes_containing_point.front());
    }

    this->lanes_in_route_ = lanes_in_route;

    smoothed_path_msg_.poses.clear();
    smoothed_path_msg_.header.frame_id = "map";
    smoothed_path_msg_.header.stamp = this->clock_->clock;

    auto routing_graph = this->map_->get_routing_graph();

    std::vector<odr::LaneKey> complete_route;

    int progress_counter = 0;
    const int ITER_LIMIT = 10;

    for (auto it = lanes_in_route.begin(); it < lanes_in_route.end() - 1; it++)
    {
        printf("%i/%i\n", progress_counter, lanes_in_route.size());
        progress_counter++;

        if (progress_counter > ITER_LIMIT)
            break;

        if (it->key.to_string() == (it + 1)->key.to_string())
        {
            // printf("Keys were the same. Skipping.\n");
            continue;
        }
        std::vector<odr::LaneKey> route = routing_graph.shortest_path((it + 1)->key, it->key);

        // Reverse the order of the route
        std::reverse(route.begin(), route.end());

        if (route.size() < 2)
        {
            // RCLCPP_WARN(this->get_logger(), "Route not found. Trying other direction.");
            route = routing_graph.shortest_path(it->key, (it + 1)->key);
            if (route.size() < 2)
            {
                RCLCPP_WARN(this->get_logger(), "Route not found.");
                continue;
            }
        }
        for (auto key : route)
        {
            if (complete_route.size() > 0 && key.to_string() == complete_route.back().to_string())
            {
                continue;
            }

            complete_route.push_back(key);
        }

        if (progress_counter % 10 == 0)
        {
            for (auto key : complete_route)
            {
                printf("%s\n", key.to_string().c_str());
            }
        }
    }

    // For each LaneKey in Route, linear sample and append to vector
    bg::model::linestring<odr::point> route_linestring;

    odr::point latest_point;

    for (auto key : complete_route)
    {
        // printf("%s\n", key.to_string().c_str());
        odr::Road road = this->map_->id_to_road.at(key.road_id);
        // printf("Road was %s\n", road.id.c_str());
        odr::LaneSection lsec = road.get_lanesection(key.lanesection_s0);
        // printf("Lsec was %f\n", lsec.s0);
        // for (auto lane : lsec.get_lanes())
        // {
        //     printf("%i ", lane.id);
        // }
        printf("\n");
        odr::Lane lane = lsec.id_to_lane.at(key.lane_id);
        // printf("Lane was %i\n", lane.id);

        odr::Line3D line = road.get_lane_border_line(lane, 1.0, true);

        // Test to see if the direction of this lane needs to be reversed
        if (route_linestring.size() > 0)
        {

            odr::point first_pt_in_lane = odr::point(line.front()[0], line.front()[1]);
            double dist = bg::distance(latest_point, first_pt_in_lane);
            printf("Distance was %f\n", dist);

            if (dist > 30) {
                std::reverse(line.begin(), line.end());
                odr::point first_pt_in_lane = odr::point(line.front()[0], line.front()[1]);
                double dist = bg::distance(latest_point, first_pt_in_lane);
                printf("Distance is now %f\n", dist);
            }
                
            
        } else {
            latest_point = odr::point(line.front()[0], line.front()[1]);
        }

        for (odr::Vec3D pt : line)
        {
            odr::point boost_point(pt[0], pt[1]);
            bg::append(route_linestring, boost_point);
            printf("Adding point (%f, %f)\n", pt[0], pt[1]);
            latest_point = boost_point;
        }
    }

    // Publish route as Path msg
    smoothed_path_msg_.header.stamp = this->clock_->clock;
    smoothed_path_msg_.header.frame_id = "map";

    smoothed_path_msg_.poses.clear();
    for (odr::point pt : route_linestring)
    {
        PoseStamped pose_msg;
        pose_msg.pose.position.x = pt.get<0>();
        pose_msg.pose.position.y = pt.get<1>();
        smoothed_path_msg_.poses.push_back(pose_msg);
    }

    route_path_pub_->publish(smoothed_path_msg_);

    RCLCPP_INFO(this->get_logger(), "Publishing path with %i poses. Started with %i poses.\n", smoothed_path_msg_.poses.size(), msg->poses.size());
}

/**
 * @brief Generate and publish the "distance from route" cost map layer
 *
 */
void navigator::perception::MapManagementNode::routeDistanceGridPubTimerCb()
{
    TransformStamped t = getVehicleTf();

    if (this->smoothed_path_msg_.poses.size() < 1) {
        RCLCPP_WARN(get_logger(), "Refined route not yet calculated. Skipping.");
        return;
    }

    route_path_pub_->publish(this->smoothed_path_msg_);
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
    this->lane_polys_ = map_->get_lane_polygons(1.0);

    RCLCPP_INFO(this->get_logger(), "Loaded %s", msg->map_name);
}
