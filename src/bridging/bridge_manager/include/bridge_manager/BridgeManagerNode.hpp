/*
 * Package:   bridge_manager
 * Filename:  BridgeManagerNode.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <string>
#include <vector>
#include "std_msgs/msg/string.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
namespace Nova {

class BridgeManagerNode : public rclcpp::Node {
public:
  BridgeManagerNode();
  virtual ~BridgeManagerNode();

private:
  void log_event(const std_msgs::msg::String::SharedPtr msg);
  void initialize();

  std::string construct(std::vector< std::array<std::string, 2> > list);

  //rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr bridge_publisher;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr bridge_subscription;

};

}
