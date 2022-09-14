/*
 * Package:   can_interface
 * Filename:  interface.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "can_interface/CanInterfaceNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if(argc < 2) { // We need an interface name to run
    std::cout << "USAGE: ros2 run can_interface_interface interface <interface>" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(std::make_shared<navigator::can_interface::CanInterfaceNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
