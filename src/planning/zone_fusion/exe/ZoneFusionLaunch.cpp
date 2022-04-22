
#include "zone_fusion/ZoneFusionNode.hpp"
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::zone_fusion::ZoneFusionNode>());
  rclcpp::shutdown();
  return 0;
}