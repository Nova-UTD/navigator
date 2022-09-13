/*
 * Package:   nova_pure_pursuit
 * Filename:  TestTrajectoryNode.cpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <chrono>
#include <memory>
#include <string>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;

using namespace std::chrono_literals;


class TestTrajectoryNode : public rclcpp::Node {

public:
  TestTrajectoryNode()
  : Node("test_trajectory_feeder"), count(0) {

    feeder = this->create_publisher
      <Trajectory>("reference_trajectory", 10);

    timer = this->create_wall_timer
      (2s, std::bind(&TestTrajectoryNode::timer_callback, this));

    this->test_trajectory = load_test_trajectory();
  }

private:
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<Trajectory>::SharedPtr feeder;
  size_t count;
  Trajectory test_trajectory;


  void timer_callback() {

    RCLCPP_INFO(this->get_logger(), "Publishing: test_trajectory(" + std::to_string(count++) + ")");
    feeder->publish(test_trajectory);
    rclcpp::shutdown(); // Publish only once
  }

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
  Trajectory load_test_trajectory() {

    Trajectory trajectory;

    std::ifstream testing_data;
    std::string username = static_cast<std::string>(getenv("USER"));
    testing_data.open("/home/" + username + "/navigator/data/TestTraj_Minidemo.txt");

    if (testing_data.is_open()) {

      std::cout << "Loading the testing file...." << std::endl;

      int i = 0;
      std::string line = "";
      while (testing_data) {

        TrajectoryPoint trajectory_point;
        testing_data >> trajectory_point.x;
        testing_data >> trajectory_point.y;
        testing_data >> trajectory_point.z;
        std::getline(testing_data, line);

        std::cout << trajectory_point.x << std::endl;
        std::cout << trajectory_point.y << std::endl;

        trajectory_point.longitudinal_velocity_mps = 0;
        
        if (!testing_data.eof()) {
          trajectory.points.push_back(trajectory_point);
          i++;
        }

      }

    }
    else { std::cout << "ERROR: Testing file is NOT found!." << std::endl; }
    testing_data.close();

    return trajectory;
  }

};


int main(int argc, char * argv[]) {

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestTrajectoryNode>());
  rclcpp::shutdown();
  return 0;
}