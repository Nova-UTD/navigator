/*
 * Package:   volron_epas_steering
 * Filename:  controller.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "voltron_epas_steering/ControllerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if(argc < 4) { // We need an interface name to run
    std::cout <<
      "USAGE: ros2 run voltron_epas_steering controller <interface>"
	      << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::spin(std::make_shared<Voltron::EpasSteering::ControllerNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
