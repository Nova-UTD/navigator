/*
 * Package:   nova_motion_control
 * Filename:  MotionControl.cpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_motion_control/MotionControlNode.hpp"

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nova::MotionControl::MotionControlNode>());
  rclcpp::shutdown();
  return 0;
}