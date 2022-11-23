/*
 * Package:   safety_manager
 * Filename:  src/safety_manager.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

// The safety manager. Currently just a hello world, but that's going to change.

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "safety_manager/SafetyManagerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nova::SafetyManager::SafetyManagerNode>());
  rclcpp::shutdown();
  return 0;
}
