#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  
  this->current_state = LANEKEEPING;

  // xml parsing
  std::string xodr_path = this->get_parameter("xodr_path").as_string();
  odr_map = new odr::OpenDriveMap(xodr_path, true, true, false, true);

  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&BehaviorPlannerNode::send_message, this));

  this->final_zone_publisher = this->create_publisher<ZoneArray>("zone_array", 10);

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&BehaviorPlannerNode::update_current_speed, this, _1));

  this->path_subscription = this->create_subscription
    <FinalPath>("paths", 8, std::bind(&BehaviorPlannerNode::update_current_path, this, _1));

  make_zones();
  
}

BehaviorPlannerNode::~BehaviorPlannerNode() {}

void BehaviorPlannerNode::update_current_path(FinalPath::SharedPtr ptr) {
  this->current_path = *ptr;
}

void BehaviorPlannerNode::update_current_speed(Odometry::SharedPtr ptr) {
  this->current_position_x = ptr->pose.pose.position.x;
  this->current_position_y = ptr->pose.pose.position.y;
  tf2::Vector3 vector_velocity(ptr->twist.twist.linear.x, ptr->twist.twist.linear.y, ptr->twist.twist.linear.z);
  this->current_speed = std::sqrt(vector_velocity.dot(vector_velocity));
}

void BehaviorPlannerNode::send_message() {

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
      if (upcoming_stop_sign()) {
        current_state = STOPPING;
      }
      break;
    case STOPPING:
      if (reached_desired_velocity(STOP_SPEED)) {
        current_state = STOPPED;
      }
      break;
    case STOPPED:
      if (!obstacles_present()) {
        current_state = LANEKEEPING;
      }
      break;
  }
}

// if path has stop sign, then create zone for it
bool BehaviorPlannerNode::upcoming_stop_sign() {
  
  zones_made = false;

  size_t m = 3; // temp horizon distance
  for(size_t i = 0; i < current_path.points.size() - m; i++) {
    
    // get road from xy points
    double current_x = current_path.points[i].x;
    double current_y = current_path.points[i].y;
    std::shared_ptr<odr::Road> road = navigator::opendrive::get_road_from_xy(odr_map, x, y);

    if (road->junction != "-1") {
      zones_made = true;
      Zone zone = navigator::zones_lib::to_zone_msg(odr_map->junctions[road->junction]);
      final_zones.push_back(zone);
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