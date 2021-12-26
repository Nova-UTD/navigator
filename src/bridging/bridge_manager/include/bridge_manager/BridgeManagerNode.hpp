/*
 * Package:   bridge_manager
 * Filename:  BridgeManagerNode.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"

namespace Nova {
namespace BridgeManager {

class BridgeManagerNode : public rclcpp::Node {
public:
  BridgeManagerNode();
  virtual ~BridgeManagerNode();

private:
  void log_event(std::string message);
};

}
}
