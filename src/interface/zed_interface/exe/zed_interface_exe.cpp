/*
 * Package:   zed_interface
 * Filename:  zed_interface_exe.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "zed_interface/ZedInterfaceNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::zed_interface::ZedInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
