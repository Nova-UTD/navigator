/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.cpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_pure_pursuit/PurePursuitNode.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;


using namespace Nova::PurePursuit;

PurePursuitNode::PurePursuitNode () : rclcpp::Node("pure_pursuit_controller"), count_(0) {  
    
    this->controller = std::make_unique<PurePursuit>(0.1); // User-defined, default lookahead
    
    this->steering_control_publisher = this->create_publisher<std_msgs::msg::String>("output/desired_steering_angle", 10);

    // timer_ = this->create_wall_timer(
    //   500ms, std::bind(&PurePursuitNode::timer_callback, this));

    this->trajectory_subscription = this->create_subscription<std_msgs::msg::String>(
      "input/reference_trajectory", 10, std::bind(&PurePursuitNode::topic_callback, this, _1));
    
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::timer_callback() {

    auto message = std_msgs::msg::String();
    message.data = controller->hello_world(); // Test Node interface
    message.data += std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    steering_control_publisher->publish(message);
 }

 void PurePursuitNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) {

  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());

  auto message = std_msgs::msg::String();
  message.data = "Trajectory received... Here's your new steering angle: ";
  message.data += std::to_string(count_++) + "\n";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  steering_control_publisher->publish(message);

 }