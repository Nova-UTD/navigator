#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>
#include <cmath>
#include <unordered_set>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  std::string xodr_path = "data/maps/town07/Town07_Opt.xodr";
  this->current_state = LANEKEEPING;

  // xml parsing
  RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
  std::shared_ptr<navigator::opendrive::MapInfo> map_info = navigator::opendrive::load_map(xodr_path);
  this->map = map_info->map;
  this->map_info = map_info;

  RCLCPP_INFO(this->get_logger(), "SIGNALS!!!%d", map_info->signals.size());
  for (const auto& [road_id, signals] : map_info->signals) {
      RCLCPP_INFO(this->get_logger(), "signals for road %s", road_id.c_str());
      for (const auto& signal : signals) {
          RCLCPP_INFO(this->get_logger(), "\tid:%s, type:%s, name:%s, s:%.2f, t:%.2f, dynamic:%s, orientation: %s", 
            signal.id.c_str(), signal.type.c_str(), signal.name.c_str(), signal.s, signal.t, signal.dynamic.c_str(), signal.orientation.c_str());
      }
  }

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
      //RCLCPP_INFO(this->get_logger(), "current state: LANEKEEPING");
      if (upcoming_intersection()) {
        //current_state = STOPPING;
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
  using navigator::opendrive::Signal;
  bool zones_made = false;
  std::unordered_set<std::string> seen_junctions;
  final_zones.zones.clear();
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

  // each path-point spaced 25 cm apart, doing 400 pts gives us total coverage of 
  // 10000 cm or 100 meters
  size_t horizon_dist = 400;
  SignalType current_signal = SignalType::None;
  for(size_t offset = 0; offset < horizon_dist; offset++) {
    size_t i = (offset + closest_pt_idx) % current_path->points.size();
    // get road that current path point is in
    double x = current_path->points[i].x;
    double y = current_path->points[i].y;
    auto lane = navigator::opendrive::get_lane_from_xy(map, x, y);
    if (lane == nullptr) {
        //RCLCPP_INFO(this->get_logger(), "(%f, %f): no road found for behavior planner", this->current_position_x, this->current_position_y);
        continue;
    }
    
    // check if road is a junction
    auto junction = lane->road.lock()->junction;
    auto id = lane->road.lock()->id;
    if (current_signal != SignalType::Stop) {
      //stop is the most restrictive, so if we aren't already under the effect of it, keep looking
      //get all signals on this road
      auto signals = map_info->signals.find(id);
      if (signals != map_info->signals.end()) {
        double s = lane->road.lock()->ref_line->match(x, y);
        for(const Signal& signal : signals->second) {
          //check if signal applies to this point
          if (navigator::opendrive::signal_applies(signal, s, id, lane->id)) {
            auto new_sig = classify_signal(signal);
            if (new_sig > current_signal) {
                //overwrite current signal because the new one is more restrictive
                current_signal = new_sig;
            }
          }
        }
      }
    }
    if (current_signal != SignalType::None && junction != "-1" && seen_junctions.find(junction) == seen_junctions.end()) {
        //we are under the effect of a signal and in a junction that we haven't seen before
        //make a zone for this junction:
        seen_junctions.insert(junction);
        zones_made = true;
        Zone zone = navigator::zones_lib::to_zone_msg(map->junctions[junction], map);
        switch(current_signal) {
            case SignalType::Yield:
                zone.max_speed = YIELD_SPEED;
                break;
            case SignalType::Stop:
                zone.max_speed = STOP_SPEED;
                break;
            default:
                zone.max_speed = 0;
                RCLCPP_WARN(this->get_logger(), "unknown signal type %d", (int)current_signal);
                break;
        }
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

BehaviorPlannerNode::SignalType BehaviorPlannerNode::classify_signal(const navigator::opendrive::Signal& signal) {
    if (signal.type == "206") {
        return SignalType::Stop;
    }
    if (signal.type == "205") {
        return SignalType::Yield;
    }
    return SignalType::None;
}