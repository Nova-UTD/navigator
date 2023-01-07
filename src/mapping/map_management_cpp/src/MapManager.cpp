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
    this->lane_polys_ = map_.get_lane_polygons(1.0);
    std::cout << "28" << std::endl;
    map_.generate_mesh_tree();
    std::cout << "30" << std::endl;
    PointMsg center;
    center.x = 50;
    center.y = -190;

    getOccupancyGrid(center, 140, 1.0);

    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::world_info_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Waiting for map...");
}

OccupancyGrid navigator::perception::MapManagementNode::getOccupancyGrid(PointMsg center, int range, float res)
{
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
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
        // std::printf("Inserting box (%f, %f)-(%f,%f)\n", b.min_corner().get<0>(), b.min_corner().get<1>(), b.max_corner().get<0>(), b.max_corner().get<1>());
    }

    std::printf("Local box has %i nodes\n", local_tree.size());

    std::ofstream f2("grid.csv");
    for (int i = x_min; i <= x_max; i++)
    {
        for (int j = y_min; j <= y_max; j++)
        {
            i = static_cast<float>(i);
            j = static_cast<float>(j);
            bool cell_is_occupied = false;
            f2 << i << ',' << j << ',';

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

                        // std::printf("Point (%f, %f) is CONTAINED within poly %i!\n", p.get<0>(), p.get<1>(), pair.second);
                        // for (auto pt : ring)
                        // {
                        //     std::printf("(%f,%f)\n", pt.get<0>(), pt.get<1>());
                        // }

                        break;
                    }
                    // else
                    // {
                    //     if (j > -180.0)
                    //     {
                    //         std::printf("Point (%f, %f) isn't contained within poly %i!\n", p.get<0>(), p.get<1>(), pair.second);
                    //         for (auto pt : ring)
                    //         {
                    //             std::printf("(%f,%f)\n", pt.get<0>(), pt.get<1>());
                    //         }
                    //     }
                    // }
                }
            }

            if (cell_is_occupied)
            {
                f2 << "1,";
            }

            else
            {
                f2 << "0,";
            }

            //         polygon poly;
            // bgt::read_wkt(
            //     "POLYGON((2 1.3,2.4 1.7,2.8 1.8,3.4 1.2,3.7 1.6,3.4 2,4.1 3,5.3 2.6,5.4 1.2,4.9 0.8,2.9 0.7,2 1.3)"
            //         "(4.0 2.0, 4.2 1.4, 4.8 1.9, 4.4 2.2, 4.0 2.0))", poly);

            f2 << std::endl;
        }
    }

    f2.close();
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() << "[ms]" << std::endl;

    return OccupancyGrid();
}
void MapManagementNode::world_info_cb(CarlaWorldInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), msg->map_name);
}
