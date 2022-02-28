/*
 * Package:   linear_actuator
 * Filename:  controller.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "linear_actuator/ControllerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::linear_actuator::ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
