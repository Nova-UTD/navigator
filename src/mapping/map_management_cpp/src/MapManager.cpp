/*
 * Package:   map_management_cpp
 * Filename:  MapManager.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

// NOTE: LOOK IN HEADER FILE FOR COMMENTS ON FUNCTION DEFINITIONS

// #include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "map_management_cpp/MapManager.hpp"

#include <algorithm>
#include <chrono>
#include <list>

#include <fstream>

using namespace navigator::perception;

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    this->lane_polys_ = map_.get_drivable_lane_polygons(1.0);
    std::cout << "28" << std::endl;
    map_.generate_mesh_tree();
    std::cout << "30" << std::endl;

    grid_pub_ = this->create_publisher<OccupancyGrid>("/grid/drivable", 10);
    clock_sub = this->create_subscription<Clock>("/clock", 10, bind(&MapManagementNode::clockCb, this, std::placeholders::_1));
    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::worldInfoCb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Waiting for map...");

    grid_pub_timer_ = this->create_wall_timer(GRID_PUBLISH_FREQUENCY, bind(&MapManagementNode::gridPubTimerCb, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

OccupancyGrid navigator::perception::MapManagementNode::getDrivableAreaGrid(PointMsg center, int range, float res)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    OccupancyGrid occupancy_grid;
    std::vector<int8_t> grid_data;

    int x_min = static_cast<int>(center.x) - range;
    int x_max = static_cast<int>(center.x) + range;
    int y_min = static_cast<int>(center.y) - range;
    int y_max = static_cast<int>(center.y) + range;
    bgi::rtree<odr::value, bgi::rstar<16, 4>> tree = this->map_.generate_mesh_tree();

    odr::box search_region(odr::point(x_min, y_min), odr::point(x_max, y_max));
    std::vector<odr::value> lane_shapes_in_range;
    tree.query(bgi::intersects(search_region), std::back_inserter(lane_shapes_in_range));

    std::printf("There are %i shapes in range.\n", lane_shapes_in_range.size());

    int idx = 0;

    bgi::rtree<odr::value, bgi::rstar<16, 4>> local_tree;

    for (unsigned i = 0; i < lane_shapes_in_range.size(); ++i)
    {
        // calculate polygon bounding box
        // insert new value
        local_tree.insert(lane_shapes_in_range.at(i));
    }

    std::printf("Local box has %i nodes\n", local_tree.size());

    std::ofstream f2("grid.csv");
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
                    odr::ring ring = this->lane_polys_.at(pair.second);
                    bool point_is_within_shape = bg::within(p, ring);
                    if (point_is_within_shape)
                    {
                        cell_is_occupied = true;

                        break;
                    }
                }
            }

            if (cell_is_occupied)
            {
                // f2 << "1,";
                grid_data.push_back(100);
            }

            else
            {
                // f2 << "0,";
                grid_data.push_back(0);
            }
        }
        f2 << std::endl;
    }

    f2.close();

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

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return occupancy_grid;
}
void navigator::perception::MapManagementNode::clockCb(Clock::SharedPtr msg)
{
    this->clock_ = msg;
}
void navigator::perception::MapManagementNode::gridPubTimerCb()
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
        return;
    }

    PointMsg center;
    center.x = t.transform.translation.x;
    center.y = t.transform.translation.y;

    OccupancyGrid msg = getDrivableAreaGrid(center, GRID_RANGE, GRID_RES);
    grid_pub_->publish(msg);
}
void MapManagementNode::worldInfoCb(CarlaWorldInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), msg->map_name);
}
