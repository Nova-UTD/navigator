/*
 * Package:   safety_manager
 * Filename:  SafetyManagerNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/srv/safety_event.hpp"

namespace Nova {
namespace SafetyManager {

using voltron_msgs::srv::SafetyEvent;

class SafetyManagerNode : public rclcpp::Node {
public:
  SafetyManagerNode();
  virtual ~SafetyManagerNode();

private:
  void log_event(const SafetyEvent::Request::SharedPtr & request, SafetyEvent::Response::SharedPtr & response);

  rclcpp::Service<SafetyEvent>::SharedPtr server;
};

}
}
