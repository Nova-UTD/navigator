/*
 * Package:   bridge_manager
 * Filename:  src/bridge_manager.cpp
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// Bridging Node for Nova

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "bridge_manager/BridgeManagerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nova::BridgeManagerNode>());
  rclcpp::shutdown();
  return 0;
}