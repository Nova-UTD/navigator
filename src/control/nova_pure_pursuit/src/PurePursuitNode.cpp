/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.cpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "nova_pure_pursuit/PurePursuitNode.hpp"
#include <chrono>
#include <memory>
#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>

using namespace std::chrono_literals;
using std::placeholders::_1;


using namespace Nova::PurePursuit;

PurePursuitNode::PurePursuitNode() : rclcpp::Node("pure_pursuit_controller") {  

  this->controller = std::make_unique
    <PurePursuit>(0.1); // User-defined, default lookahead
  
  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&PurePursuitNode::send_message, this));

  this->steering_control_publisher = this->create_publisher
    <SteeringAngle>("steering_angle", 10);
  
  this->trajectory_subscription = this->create_subscription
    <Trajectory>("reference_trajectory", 8, std::bind(&PurePursuitNode::update_trajectory, this, _1));

  std::vector<autoware_auto_msgs::msg::TrajectoryPoint> test_trajectory;
  load_test_trajectory(test_trajectory);
}

PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::send_message() {

  /* TODO:
  - Select lookahead & closest points using util functions
  - set new points for the controller
  - This must be done regardless of whether having received a new trajectory 
  */

  auto angle = controller->get_steering_angle();

  // format of published message
  auto steering_angle_msg = SteeringAngle();
  steering_angle_msg.steering_angle = angle;

  RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", angle);
  steering_control_publisher->publish(steering_angle_msg);
}

void PurePursuitNode::update_trajectory(Trajectory::SharedPtr ptr) {

  RCLCPP_INFO(this->get_logger(), "Trajectory received");
  auto trajectory = ptr->points;
}

// TODO: Move this function to a test-providing node
/**********************************************************************************
* Function:     load_test_trajectory()
*
* Descr:        Parses an input txt file of xyz points to build a sample trajectory
*               Format:
*                   x
*                   y
*                   z
*                   [empty line]
**********************************************************************************/
void PurePursuitNode::load_test_trajectory(std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &v) {

  std::ifstream testing_data;
  std::string username = static_cast<std::string>(getenv("USER"));
  testing_data.open("/home/" + username + "/navigator/data/TrajTest1_Gomentum(unformated).txt");

  if (testing_data.is_open()) {

    std::cout << "Loading the testing file...." << std::endl;

    std::string line = "";
    while (testing_data) {

      std::getline(testing_data, line);
      autoware_auto_msgs::msg::TrajectoryPoint trajectory_point;
      testing_data >> trajectory_point.x;
      testing_data >> trajectory_point.y;
      testing_data >> trajectory_point.z;

      trajectory_point.longitudinal_velocity_mps = 0;
      
      if (!testing_data.eof()) {
        v.push_back(trajectory_point);
      }

    }

  }
  else { std::cout << "ERROR: Testing file is NOT found!." << std::endl; }
  testing_data.close();
}