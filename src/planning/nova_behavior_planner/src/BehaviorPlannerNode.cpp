#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  this->declare_parameter<std::string>("xodr_path", "/home/main/navigator/data/maps/town10/Town10HD_Opt.xodr");
  this->declare_parameter<double>("path_resolution", 2.0);
  this->current_state = LANEKEEPING;

  // xml parsing
  std::string xodr_path = this->get_parameter("xodr_path").as_string();
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
  }

  final_zone_publisher->publish(this->final_zones);
}

void BehaviorPlannerNode::update_state() {
  switch(current_state) {
    case LANEKEEPING:
      RCLCPP_INFO(this->get_logger(), "current state: LANEKEEPING");
      if (upcoming_stop_sign()) {
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
  }
}

// if path has stop sign, then create zone for it
bool BehaviorPlannerNode::upcoming_stop_sign() {
  bool zones_made = false;

  size_t m = 3; // temp horizon distance
  for(size_t i = 0; i < current_path->points.size() - m; i++) {
    
    // get road from xy points
    std::shared_ptr<const odr::Road> road = navigator::opendrive::get_road_from_xy(odr_map, this->current_position_x, this->current_position_y);
    if (road == nullptr) {
        RCLCPP_INFO(this->get_logger(), "(%f, %f): no road found for behavior planner", this->current_position_x, this->current_position_y);
        return false;
    }
    
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