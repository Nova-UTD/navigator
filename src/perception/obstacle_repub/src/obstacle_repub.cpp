/*
 * Package:   obstacle_repub
 * Filename:  obstacle_repub.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "obstacle_repub/obstacle_repub_node.hpp"
#include <memory>


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::obstacle_repub::ObstacleRepublisher>());
  rclcpp::shutdown();
  return 0;
}
