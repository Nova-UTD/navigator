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

#include <list>
#include <algorithm>

#include <fstream>

using namespace navigator::perception;

MapManagementNode::MapManagementNode() : Node("map_management_node")
{
    this->lane_polys_ = map_.get_lane_polygons(1.0);
    std::cout << "28" << std::endl;
    map_.generate_mesh_tree();
    std::cout << "30" << std::endl;
    PointMsg center;
    center.x = 0;
    center.y = 0;
    // getOccupancyGrid(center, 400, 1.0);

    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::world_info_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Waiting for map...");
}

OccupancyGrid navigator::perception::MapManagementNode::getOccupancyGrid(PointMsg center, int range, float res)
{
    std::ofstream f("points.csv");
    std::cout << "42" << std::endl;
    int x_min = static_cast<int>(center.x) - range;
    int x_max = static_cast<int>(center.x) + range;
    int y_min = static_cast<int>(center.y) - range;
    int y_max = static_cast<int>(center.y) + range;
    bgi::rtree<odr::value, bgi::rstar<16, 4>> tree = this->map_.generate_mesh_tree();

    std::cout << "50" << std::endl;
    std::printf("Tree has %i nodes\n", tree.size());

    for (float i = x_min; i <= x_max; i += res)
    {
        for (float j = y_min; j <= y_max; j += res)
        {
            std::vector<odr::value> result_n;
            auto pt = odr::point(i, j);

            bool within_lane = false;

            for (auto it = tree.qbegin(bgi::covers(pt)); it != tree.qend(); it++)
            {
                odr::value result = *it;
                auto poly = this->lane_polys_.at(result.second);
                if (bg::intersects(pt, poly))
                {
                    within_lane = true;
                    std::printf("Point intersects!: \n");
                }
            }

            if (within_lane)
                f << "1,";
            else
                f << "0,";

            // int result_count = tree.query(bgi::contains(), std::back_inserter(result_n));
            // bool point_is_within_lane = false;
            // if (result_count >= 1)
            // {
            //     // Our point (i,j) falls within a box in the tree
            //     // This only means that it falls within the bounding box of a lane,
            //     // not necessarily that it falls within the lane itself.
            //     std::printf("Point has %i results\n", result_n.size());
            //     for (auto result : result_n)
            //     {
            //         unsigned result_idx = result.second;
            //         auto poly = this->lane_polys_.at(result_idx);
            //     }
            //     if (point_is_within_lane)
            //         f << "1,";
            //     else
            //         f << "0,";
            // }
            // else
            // {
            //     f << "0,";
            // }
        }
        f << std::endl;
    }

    return OccupancyGrid();
}
void MapManagementNode::world_info_cb(CarlaWorldInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), msg->map_name);
}
