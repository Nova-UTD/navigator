
#include "obstacle_zoner/ObstacleZonerNode.hpp"
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::obstacle_zoner::ObstacleZonerNode>());
  rclcpp::shutdown();
  return 0;
}