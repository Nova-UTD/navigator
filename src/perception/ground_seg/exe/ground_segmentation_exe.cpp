/*
 * Package:   occupancy_cpp
 * Filename:  ground_segmentation_exe.cpp
 * Author:    Will Heitman
 * Email:     w at heit dot mn
 * Copyright: 2023, Nova UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "ground_seg/GroundSegmentationNode.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<navigator::perception::GroundSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
