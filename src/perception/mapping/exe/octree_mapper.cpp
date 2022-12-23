/*
 * Package:   mapping
 * Filename:  octree_mapper.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "mapping/OctreeMappingNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<navigator::perception::OctreeMappingNode>());
  rclcpp::shutdown();
  return 0;
}
