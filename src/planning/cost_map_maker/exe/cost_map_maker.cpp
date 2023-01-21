/*
 * Package:   cost_map_maker
 * Filename:  cost_map_maker.cpp
 * Author:    Chitsein Htun
 * Email:     chtun@live.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "cost_map_maker/CostMapMakerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::cost_map_maker::CostMapMakerNode>());
  rclcpp::shutdown();
  return 0;
}
