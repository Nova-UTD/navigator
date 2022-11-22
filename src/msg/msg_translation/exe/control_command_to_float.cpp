/*
 * Package:   msg_translation
 * Filename:  exe/control_command_to_float.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "msg_translation/ControlCommandToFloatNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::msg_translation::ControlCommandToFloatNode>());
  rclcpp::shutdown();
  return 0;
}
