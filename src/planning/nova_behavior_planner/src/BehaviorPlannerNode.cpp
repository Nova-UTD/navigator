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

    // todo
    //this->controller = std::make_unique<PurePursuit>(0.1);
  
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
  
    CostedPath chosen_path;
    for(size_t i = 0; i < costed_paths.paths.size(); i++) {
        float total_cost = costed_paths.paths[i].safety_cost + costed_paths.paths[i].routing_cost;
        if (total_cost < chosen_path.safety_cost + chosen_path.routing_cost) {
            chosen_path = costed_paths.paths[i];    
        }
    }

    final_path.points = chosen_path.points;
    final_path_publisher->publish(this->final_path);
}

void BehaviorPlannerNode::update_paths(CostedPaths::SharedPtr ptr) {
  RCLCPP_INFO(this->get_logger(), "Paths received!");
  this->costed_paths = *ptr;
}

void BehaviorPlannerNode::update_current_position(Odometry::SharedPtr ptr) {
  this->current_position = *ptr;
}

void BehaviorPlannerNode::update_state() {

}
