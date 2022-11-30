/*
 * Package:   motion_planner
 * Filename:  motion_planner.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "motion_planner/MotionPlannerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::motion_planner::MotionPlannerNode>());
  rclcpp::shutdown();
  return 0;
}
