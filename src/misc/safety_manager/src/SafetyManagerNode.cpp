/*
 * Package:   safety_manager
 * Filename:  SafetyManagerNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "safety_manager/SafetyManagerNode.hpp"

using namespace Nova::SafetyManager;
using voltron_msgs::srv::SafetyEvent;

SafetyManagerNode::SafetyManagerNode() : Node("safety_manager") {
  this->server = this->create_service<SafetyEvent>("safety_events",
    [&] (const SafetyEvent::Request::SharedPtr request, SafetyEvent::Response::SharedPtr response) {
      this->log_event(request, response);
    }
  );
}

SafetyManagerNode::~SafetyManagerNode() {

}

void SafetyManagerNode::log_event(const SafetyEvent::Request::SharedPtr & request, SafetyEvent::Response::SharedPtr & response) {
  RCLCPP_INFO(this->get_logger(), "Got a safety event!");
}
