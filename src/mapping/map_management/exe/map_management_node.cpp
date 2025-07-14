/*
 * Package:   map_management
 * Filename:  map_management_node.cpp
 * Author:    Will Heitman
 * Email:     project.nova@utdallas.edu
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "map_management/MapManager.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::planning::MapManagementNode>());
  rclcpp::shutdown();
  return 0;
}
