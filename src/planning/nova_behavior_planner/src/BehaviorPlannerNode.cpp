#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  std::string xodr_path = "data/maps/town07/Town07_Opt.xodr";
  this->current_state = LANEKEEPING;

  // xml parsing
  RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
  this->odr_map = new odr::OpenDriveMap(xodr_path, {true, true, true, false, true});

  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&BehaviorPlannerNode::send_message, this));

  this->final_zone_publisher = this->create_publisher<ZoneArray>("zone_array", 10);

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&BehaviorPlannerNode::update_current_speed, this, _1));

  this->path_subscription = this->create_subscription
    <FinalPath>("paths", 8, std::bind(&BehaviorPlannerNode::update_current_path, this, _1));
  
}

BehaviorPlannerNode::~BehaviorPlannerNode() {}

void BehaviorPlannerNode::update_current_path(FinalPath::SharedPtr ptr) {
  this->current_path = ptr;
}

void BehaviorPlannerNode::update_current_speed(Odometry::SharedPtr ptr) {
  this->current_position_x = ptr->pose.pose.position.x;
  this->current_position_y = ptr->pose.pose.position.y;
  tf2::Vector3 vector_velocity(ptr->twist.twist.linear.x, ptr->twist.twist.linear.y, ptr->twist.twist.linear.z);
  this->current_speed = std::sqrt(vector_velocity.dot(vector_velocity));
}

void BehaviorPlannerNode::send_message() {
  if (this->current_path == nullptr) return;
  // update state based on conditions
  
  update_state();

  // in case extra behavior needed
  switch(current_state) {
    case LANEKEEPING:
      break;
    case STOPPING:
      break;
    case STOPPED:
      break;
    case IN_INTERSECTION:
      break;
  }

  final_zone_publisher->publish(this->final_zones);
}

void BehaviorPlannerNode::update_state() {
  switch(current_state) {
    case LANEKEEPING:
      RCLCPP_INFO(this->get_logger(), "current state: LANEKEEPING");
      if (upcoming_intersection()) {
        current_state = STOPPING;
      }
      break;
    case STOPPING:
      RCLCPP_INFO(this->get_logger(), "current state: STOPPING");
      if (reached_desired_velocity(STOP_SPEED)) {
        current_state = STOPPED;
      }
      break;
    case STOPPED:
      RCLCPP_INFO(this->get_logger(), "current state: STOPPED");
      if (!obstacles_present()) {
        current_state = LANEKEEPING;
      }
      break;
    case IN_INTERSECTION:
      RCLCPP_INFO(this->get_logger(), "current state: IN_INTERSECTION");
      // if (!in_zone()) {
      // }
  }
}

// if path has intersection, then create zone for it
bool BehaviorPlannerNode::upcoming_intersection() {
  bool zones_made = false;

  // find point on path closest to current location
  size_t closest_pt_idx = 0;
  float min_distance = -1;
  for (size_t i = 0; i < current_path->points.size(); i++) {
    float distance = pow(current_path->points[i].x - current_position_x, 2);
    distance += pow(current_path->points[i].y - current_position_y, 2); 
    if (min_distance == -1 || distance < min_distance) {
      min_distance = distance;
      closest_pt_idx = i;
    }
  }

  // each path-point spaced 25 cm apart, doing 200 pts gives us total coverage of 
  // 5000 cm or 50 meters
  size_t horizon_dist = 200;
  for(size_t i = closest_pt_idx; i < closest_pt_idx + horizon_dist; i++) {
    
    // get road that current path point is in
    std::shared_ptr<const odr::Road> road = navigator::opendrive::get_road_from_xy(odr_map, current_path->points[i].x, current_path->points[i].y);
    if (road == nullptr) {
        RCLCPP_INFO(this->get_logger(), "(%f, %f): no road found for behavior planner", this->current_position_x, this->current_position_y);
        return false;
    }
    
    // check if road is a junction
    auto junction = road->junction;
    if (junction != "-1") {
      RCLCPP_INFO(this->get_logger(), "in junction " + junction);
      zones_made = true;
      Zone zone = navigator::zones_lib::to_zone_msg(this->get_logger(), odr_map->junctions[junction], odr_map);
      final_zones.zones.push_back(zone);
    }
  }

  return zones_made;
}

// no perception data so just returning false for now
bool BehaviorPlannerNode::obstacles_present() {
  return false;
}

bool BehaviorPlannerNode::reached_desired_velocity(float desired_velocity) {
  return desired_velocity >= current_speed;
}