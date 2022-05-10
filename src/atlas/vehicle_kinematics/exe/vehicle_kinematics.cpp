/*
 * Package:   vehicle_kinematics
 * Filename:  exe/vehicle_kinematics.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "vehicle_kinematics/VehicleKinematicsNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::vehicle_kinematics::VehicleKinematicsNode>());
  rclcpp::shutdown();
  return 0;
}
