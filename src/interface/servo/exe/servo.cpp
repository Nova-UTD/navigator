/*
 * Package:   servo
 * Filename:  exe/servo.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "servo/ServoNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::servo::ServoNode>());
  rclcpp::shutdown();
}
