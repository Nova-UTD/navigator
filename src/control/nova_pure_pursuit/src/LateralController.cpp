/*
 * Package:   nova_pure_pursuit
 * Filename:  LateralController.cpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_pure_pursuit/PurePursuitNode.hpp"

#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"






int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nova::PurePursuit::PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}