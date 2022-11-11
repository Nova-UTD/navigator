/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

//NOTE: LOOK IN HEADER FILE FOR COMMENTS ON FUNCTION DEFINITIONS

//#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include <boost/geometry.hpp>

#include "cost_map_maker/CostMapMakerNode.hpp"

#include <list>
#include <algorithm>

using namespace navigator::cost_map_maker;

using voltron_msgs::msg::Egma;
using voltron_msgs::msg::EvidentialGrid;
using geometry_msgs::msg::Vector3;

CostMapMakerNode::CostMapMakerNode() : Node("cost_map_maker_node")
{
    std::string xodr_path = "data/maps/town07/Town07_Opt.xodr";
    
    cost_map_publisher = this->create_publisher<voltron_msgs::msg::Egma>("outgoing_cost_map", 8);
    egma_subscription = this->create_subscription<voltron_msgs::msg::Egma>("*INSERT SUBSCRIPTION*", 10, bind(&CostMapMakerNode::update_egma, this, std::placeholders::_1));
    waypoint_subscription = this->create_subscription<geometry_msgs::msg::Vector3>("*INSERT SUBSCRIPTION*", 10, bind(&CostMapMakerNode::update_waypoints, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&CostMapMakerNode::odometry_pose_cb, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&CostMapMakerNode::send_message, this));
    
    // Read map from file, using our path param
	RCLCPP_INFO(this->get_logger(), "Reading from ", xodr_path.c_str());
	carla_map = navigator::opendrive::load_map(xodr_path)->map;
}

void CostMapMakerNode::send_message() {
    if (egma == nullptr) {
        return;
    }

    auto cost_map = voltron_msgs::msg::Egma();
    for (size_t time_index = 0; time_index < egma->egma.size(); time_index++) {
      auto cost_map_single_timestep = voltron_msgs::msg::EvidentialGrid();
      auto current_evidential_grid = egma->egma[time_index];
      cost_map_single_timestep.header = current_evidential_grid.header;
      for (size_t coordinate_index = 0; coordinate_index < current_evidential_grid.grid.size(); coordinate_index++)
      {
        cost_map_single_timestep.grid[coordinate_index].occupancy_value = current_evidential_grid.grid[coordinate_index].occupancy_value;
        cost_map_single_timestep.grid[coordinate_index].occupancy_value = current_evidential_grid.grid[coordinate_index].occupancy_value;
      }
      cost_map.egma.push_back(cost_map_single_timestep);
    }
    cost_map_publisher->publish(cost_map);


    return;
}

void CostMapMakerNode::update_egma(voltron_msgs::msg::Egma::SharedPtr ptr) {
    egma = ptr;
}

void CostMapMakerNode::update_waypoints(geometry_msgs::msg::Vector3::SharedPtr ptr)
{
    waypoints = ptr;
}

void CostMapMakerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry = msg;
}