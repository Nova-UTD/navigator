/*
 * Package:   volron_steering_pid
 * Filename:  pid_controller.cpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#include <iostream>
#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "pid_controller/PidControllerNode.hpp"
#include "pid_controller/PidController.hpp"
#include <string>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Voltron::PidController::PidControllerNode>());
  rclcpp::shutdown();
  return 0;
}
