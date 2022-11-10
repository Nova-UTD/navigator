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
#include "voltron_msgs/msg/zone.hpp"
#include "zone_lib/zone.hpp"

#include <list>
#include <algorithm>

using namespace navigator::cost_map_maker;
using namespace navigator::zones_lib;

using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;
using voltron_msgs::msg::Egma;
using voltron_msgs::msg::EvidentialGrid;


CostMapMakerNode::CostMapMakerNode() : Node("cost_map_maker_node")
{
    cost_map_publisher = this->create_publisher<voltron_msgs::msg::Egma>("outgoing_cost_map", 8);
    egma_subscription = this->create_subscription<voltron_msgs::msg::Egma>("*INSERT SUBSCRIPTION*", 10, bind(&MotionPlannerNode::update_egma, this, std::placeholders::_1));
    waypoint_subscription = this->create_subscription<ZoneArray>("*INSERT SUBSCRIPTION*", 10, bind(&MotionPlannerNode::update_waypoints, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    //current_pose_subscription = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    //steering_angle_subscription = this->create_subscription<voltron_msgs::msg::SteeringPosition>("/can/steering_angle", 8, bind(&MotionPlannerNode::update_steering_angle, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
    //planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    if (egma == nullptr) {
        return;
    }

    auto cost_map = voltron_msgs::msg::Egma();
    for (size_t time_index = 0; time_index < egma->egma.size(); time_index++) {
      auto cost_map_single_timestep = voltron_msgs::msg::EvidentialGrid();
      auto current_evidential_grid = egma->egma[time_index];
      cost_map_single_timestep.timestep = current_evidential_grid.timestep;
      for (size_t coordinate_index = 0; coordinate_index < current_evidential_grid.grid.size(); coordinate_index++)
      {
        cost_map_single_timestep.grid[coordinate_index].occupancy_value = current_evidential_grid.grid[coordinate_index].occupancy_value;
        cost_map_single_timestep.grid[coordinate_index].occupancy_value = current_evidential_grid.grid[coordinate_index].occupancy_value;
      }
      cost_map.egma.push_back(cost_map_single_timestep);
    }
    trajectory_publisher->publish(cost_map_egma);


    return;
}

void MotionPlannerNode::update_egma(voltron_msgs::msg::Egma::SharedPtr ptr) {
    egma = ptr;
}

void MotionPlannerNode::update_waypoints(voltron_msgs::msg::ZoneArray::SharedPtr ptr)
{

}

void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry = msg;
}