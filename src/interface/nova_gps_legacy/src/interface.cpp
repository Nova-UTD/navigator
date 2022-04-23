/*
 * Package:   nova_gps
 * Filename:  interface.cpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "nova_gps/GPSInterfaceNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if(argc < 2) { // We need an interface name to run
    std::cout << "USAGE: ros2 run nova_gps_interface interface <interface>" << std::endl;
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(std::make_shared<Nova::GPS::GPSInterfaceNode>(argv[1]));
  rclcpp::shutdown();
  return 0;
}
