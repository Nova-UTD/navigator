/*
 * Package:   msg_translation
 * Filename:  exe/velocity_to_twist.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "msg_translation/VelocityToTwistNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::msg_translation::VelocityToTwistNode>());
  rclcpp::shutdown();
  return 0;
}
