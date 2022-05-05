/*
 * Package:   curb_localizer
 * Filename:  curb_localizer_exe.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "curb_localizer/CurbLocalizerNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::curb_localizer::CurbLocalizerNode>());
  rclcpp::shutdown();
  return 0;
}