/*
 * Package:   safety_manager
 * Filename:  SafetyManagerNode.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "safety_manager/SafetyManagerNode.hpp"

#include <string>

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
  std::string message;
  switch(request->status) {
  case SafetyEvent::Request::STATUS_WORKING:
    message += "A node is working to resolve safety event: ";
    break;
  case SafetyEvent::Request::STATUS_UNRESOLVED:
    message += "A safety event is unresolved: ";
    break;
  case SafetyEvent::Request::STATUS_RESOLVED:
    message += "A safety event has been resolved: ";
    break;
  default:
    message += "A safety event was raised with a bad status: ";
    break;
  }
  message += request->description;
  RCLCPP_INFO(this->get_logger(), message);

  (void) response;
}
