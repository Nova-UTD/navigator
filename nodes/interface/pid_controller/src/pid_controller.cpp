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
#include "voltron_steering_pid/PidControllerNode.hpp"
#include "voltron_steering_pid/PidController.hpp"
#include <string>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  if(argc < 4) {
    std::cout << "USAGE: ros2 run voltron_steering_pid pid_controller <P> <I> <D>" << std::endl;
    rclcpp::shutdown();
    return 1;
  }

  float P = std::stof(argv[1]);
  float I = std::stof(argv[2]);
  float D = std::stof(argv[3]);

  auto pid_controller = std::make_unique<Voltron::SteeringPid::PidController>
    (P, I, D, 0.2);
  
  rclcpp::spin(std::make_shared<Voltron::SteeringPid::PidControllerNode>
	       (std::move(pid_controller)));
  rclcpp::shutdown();
  return 0;
}
