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

using namespace navigator::perception;

MapManagementNode::MapManagementNode() : Node("map_management_node")
{

    // odr::RoadNetworkMesh mesh = map_.get_road_network_mesh(1.0);
    auto polys = map_.get_road_polygons(1.0);
    map_.generate_mesh_tree();
    RCLCPP_INFO(this->get_logger(), std::to_string(polys.size()));

    world_info_sub = this->create_subscription<CarlaWorldInfo>("/carla/world_info", 10, bind(&MapManagementNode::world_info_cb, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Waiting for map...");
}

void MapManagementNode::world_info_cb(CarlaWorldInfo::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), msg->map_name);
}
