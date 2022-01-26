/*
 * Package:   can_translation
 * Filename:  exe/float_reporter.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "can_translation/FloatReporterNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::can_translation::FloatReporterNode>());
  rclcpp::shutdown();
  return 0;
}
