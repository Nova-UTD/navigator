/*
 * Package:   lidar_obstacle_detector
 * Filename:  lidar_obstacle_detector_node_exe.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "lidar_obstacle_detector/LidarObstacleDetectorNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::lidar_obstacle_detector::LidarObstacleDetectorNode>());
  rclcpp::shutdown();
  return 0;
}