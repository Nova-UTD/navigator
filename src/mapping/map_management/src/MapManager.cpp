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
    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));

    grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::gridPubTimerCb, this));
    route_update_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::updateRoute, this));

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

    int x_min = static_cast<int>(center.x) - range;
    int x_max = static_cast<int>(center.x) + range;
    int y_min = static_cast<int>(center.y) - range;
    int y_max = static_cast<int>(center.y) + range;
    this->map_wide_tree_ = this->map_->generate_mesh_tree();

    odr::box search_region(odr::point(x_min, y_min), odr::point(x_max, y_max));
    std::vector<odr::value> lane_shapes_in_range;
    map_wide_tree_.query(bgi::intersects(search_region), std::back_inserter(lane_shapes_in_range));

    std::printf("There are %i shapes in range.\n", lane_shapes_in_range.size());

    int idx = 0;

    bgi::rtree<odr::value, bgi::rstar<16, 4>> local_tree;

    for (unsigned i = 0; i < lane_shapes_in_range.size(); ++i)
        local_tree.insert(lane_shapes_in_range.at(i));

    for (int j = y_min; j <= y_max; j++)
    {
        for (int i = x_min; i <= x_max; i++)
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
        }
    }

    // Set the OccupancyGrid's metadata
    auto clock = this->clock_->clock;
    occupancy_grid.data = grid_data;
    occupancy_grid.header.frame_id = "map";
    occupancy_grid.header.stamp = clock;
    occupancy_grid.info.width = range * 2 + 1;
    occupancy_grid.info.height = range * 2 + 1;
    occupancy_grid.info.map_load_time = clock;
    occupancy_grid.info.resolution = GRID_RES;
    occupancy_grid.info.origin.position.x = x_min;
    occupancy_grid.info.origin.position.y = y_min;

    // Output function runtime
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

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
void navigator::perception::MapManagementNode::gridPubTimerCb()
{
    TransformStamped t = getVehicleTf();

    PointMsg center;
    center.x = t.transform.translation.x;
    center.y = t.transform.translation.y;

    OccupancyGrid msg = getDrivableAreaGrid(center, GRID_RANGE, GRID_RES);
    grid_pub_->publish(msg);
}

void navigator::perception::MapManagementNode::getLanesFromRouteMsg(CarlaRoute::SharedPtr msg)
{
    if (this->lanes_in_route_.size() > 0)
        return;

    std::vector<odr::Lane> lanes_in_route;

    for (auto pose : msg->poses) {
        // RCLCPP_INFO()
    }
    RCLCPP_INFO(this->get_logger(), "Received %i poses along route.");
}

/**
 * @brief Generate and publish the "distance from route" cost map layer
 *
 */
void navigator::perception::MapManagementNode::updateRoute()
{
    TransformStamped t = getVehicleTf();

    // TODO: Get the lane that the car is currently in

    // TODO: Print the lane's ID and road ID
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
