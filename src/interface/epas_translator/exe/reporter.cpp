/*
 * Package:   epas_translator
 * Filename:  reporter.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "epas_translator/ReporterNode.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Voltron::EpasSteering::ReporterNode>());
  rclcpp::shutdown();
  return 0;
}
