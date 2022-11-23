
#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Nova::BehaviorPlanner::BehaviorPlannerNode>());
  rclcpp::shutdown();
  return 0;
}