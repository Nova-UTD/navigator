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
using voltron_msgs::msg::Dogma;
using voltron_msgs::msg::Ogma;


CostMapMakerNode::CostMapMakerNode() : Node("cost_map_maker_node")
{
    cost_map_publisher = this->create_publisher<voltron_msgs::msg::Dogma>("outgoing_cost_map", 8);
    DOGMa_subscription = this->create_subscription<voltron_msgs::msg::Dogma>("*INSERT SUBSCRIPTION*", 10, bind(&MotionPlannerNode::update_DOGMa, this, std::placeholders::_1));
    waypoint_subscription = this->create_subscription<ZoneArray>("*INSERT SUBSCRIPTION*", 10, bind(&MotionPlannerNode::update_waypoints, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    //current_pose_subscription = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    //steering_angle_subscription = this->create_subscription<voltron_msgs::msg::SteeringPosition>("/can/steering_angle", 8, bind(&MotionPlannerNode::update_steering_angle, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
    //planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    if (DOGMa == nullptr) {
        return;
    }

    auto cost_map_DOGMa = voltron_msgs::msg::Dogma();
    for (size_t time_index = 0; time_index < DOGMA->DOGMa.size(); time_index++) {
      auto cost_map_OGMa = voltron_msgs::msg::Ogma();
      auto cur_OGMa = DOGMa->DOGMa[time_index];
      cost_map_OGMa.timestep = cur_OGMa.timestep;
      for (size_t coordinate_index = 0; coordinate_index < cur_OGMa.map.size(); coordinate_index++)
      {
        cost_map_OGMa.map[coordinate_index].value = cur_OGMa.map[coordinate_index].value;
        cost_map_OGMa.map[coordinate_index].value = cur_OGMa.map[coordinate_index].value;
      }
      cost_map_DOGMa.DOGMa.push_back(cost_map_OGMa);
    }
    trajectory_publisher->publish(cost_map_DOGMa);


    return;
}

void MotionPlannerNode::update_DOGMa(voltron_msgs::msg::Dogma::SharedPtr ptr) {
    DOGMa = ptr;
}

void MotionPlannerNode::update_waypoints(voltron_msgs::msg::ZoneArray::SharedPtr ptr)
{

}

void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry = msg;
}