#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

using FinalPath = voltron_msgs::msg::FinalPath;
using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  
  this->current_state = LANEKEEPING;

  this->behavior_planner = std::make_unique<BehaviorPlanner>();
  
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&BehaviorPlannerNode::send_message, this));

  this->final_path_publisher = this->create_publisher
    <FinalPath>("final_path", 10);

  this->paths_subscription = this->create_subscription
    <CostedPaths>("paths", 8, std::bind(&BehaviorPlannerNode::update_paths, this, _1));

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&BehaviorPlannerNode::update_current_position, this, _1));
}

BehaviorPlannerNode::~BehaviorPlannerNode() {}

void BehaviorPlannerNode::send_message() {

  // CHOOSE LOWEST COST  
  CostedPath chosen_path;
  for(size_t i = 0; i < costed_paths.paths.size(); i++) {
      float total_cost = costed_paths.paths[i].safety_cost + costed_paths.paths[i].routing_cost;
      if (total_cost < chosen_path.safety_cost + chosen_path.routing_cost) {
          chosen_path = costed_paths.paths[i];    
      }
  }

  final_path.points = chosen_path.points;
  for(size_t i = 0; i < final_path.points.size(); i++) {
      final_path.speeds.push_back(SPEED_LIMIT);
  }
  
  // CHOOSE COST BASED OFF STATE MACHINE
  // will modify final path variable to publish
  update_state();
  
  final_path_publisher->publish(this->final_path);
}

void BehaviorPlannerNode::update_paths(CostedPaths::SharedPtr ptr) {
  RCLCPP_INFO(this->get_logger(), "Paths received!");
  this->costed_paths = *ptr;
}

void BehaviorPlannerNode::update_current_position(Odometry::SharedPtr ptr) {
  this->current_position = *ptr;
}

bool BehaviorPlannerNode::reached_desired_velocity(float desired_velocity) {
  // get vehicle kinematic state and check if velocity is equal
  return desired_velocity == 0.0;
}

void BehaviorPlannerNode::update_state() {

  switch(current_state) {
    case LANEKEEPING:
      if (behavior_planner->immediate_collision()) {
        current_state = EMERGENCYSTOP;
      } else if (behavior_planner->upcoming_stop_sign()) {
        current_state = STOPPING;
      } else if (behavior_planner->upcoming_yield_sign()) {
        current_state = YIELDING;
      } else if (behavior_planner->upcoming_speedbump()) {
        current_state = SLOWINGDOWN;
      } else if (behavior_planner->destination_close()) {
        current_state = STOPPING;
      }
      break;
    case STOPPING:
      if (behavior_planner->immediate_collision()) {
        current_state = EMERGENCYSTOP;
      } else if (reached_desired_velocity(STOP_SPEED)) {
        current_state = STOPPED;
      }
      break;
    case YIELDING:
      if (behavior_planner->immediate_collision()) {
        current_state = EMERGENCYSTOP;
      } else if (behavior_planner->upcoming_stop_sign()) {
        current_state = STOPPING;
      } else if (reached_desired_velocity(YIELD_SPEED)) {
        current_state = LANEKEEPING;
      }
      break;
    case SLOWINGDOWN:
      if (behavior_planner->immediate_collision()) {
        current_state = EMERGENCYSTOP;
      } else if (reached_desired_velocity(SLOW_SPEED)) {
        current_state = LANEKEEPING;
      }
      break;
    case STOPPED:
      if (!behavior_planner->obstacles_present()) {
        current_state = LANEKEEPING;
      }
      break;
    case EMERGENCYSTOP:
      if (reached_desired_velocity(STOP_SPEED)) {
        current_state = LANEKEEPING; // change??
      }
      break;
    default:
      break;
  }



}
