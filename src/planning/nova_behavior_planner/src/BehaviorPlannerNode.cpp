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

}

BehaviorPlannerNode::~BehaviorPlannerNode() {}


void BehaviorPlannerNode::update_current_speed(Odometry::SharedPtr ptr) {
  tf2::Vector3 vector_velocity(ptr->twist.twist.linear.x, ptr->twist.twist.linear.y, ptr->twist.twist.linear.z);
  this->current_speed = std::sqrt(vector_velocity.dot(vector_velocity));
}

void BehaviorPlannerNode::send_message() {

  // update state based on conditions
  update_state();

  switch(current_state) {
    case LANEKEEPING:
      break;
    case STOPPING:
      add_stop_zone();
      break;
    case STOPPED:
      add_stop_zone();
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


// there is an intersection so we find where to add a zone
// and add that zone to zone array
void BehaviorPlannerNode::add_stop_zone() {

}


// checks if intersection ahead, adds zone for intersection to array if needed
bool BehaviorPlannerNode::upcoming_stop_sign() {
  
  std::shared_ptr<odr::Road> prev_road;
  // for(size_t i = 0; i < final_path.points.size(); i++) {
  //   std::shared_ptr<odr::Road> road = odr_map->get_road_from_xy(final_path.points[i].x, final_path.points[i].y);
    
  //   // same pointers
  //   if (prev_road == road) {
  //     continue;
  //   }

  //   // xml parse to tell if road has a signal
  //   if(odr_map->road_has_signals(road)) {
  //     stopping_point_idx = i;
  //   }
  // }

  return true;
}

// no perception data so just returning false for now
bool BehaviorPlannerNode::obstacles_present() {
  return false;
}

bool BehaviorPlannerNode::reached_desired_velocity(float desired_velocity) {
  return desired_velocity >= current_speed;
}