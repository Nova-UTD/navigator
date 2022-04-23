/*
 * Package:   joy_control
 * Filename:  exe/joy_control.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "joy_control/JoyControlNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::joy_control::JoyControlNode>());
  rclcpp::shutdown();
}
