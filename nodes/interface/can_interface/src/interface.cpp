/*
 * Package:   can_interface
 * Filename:  interface.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "can_interface/CanInterfaceNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  /*std::string interface_name;
  rclcpp::NodeHandle node_handle;
  node_handle.getParam("interface_name", interface_name);*/
  rclcpp::spin(std::make_shared<Voltron::Can::CanInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
