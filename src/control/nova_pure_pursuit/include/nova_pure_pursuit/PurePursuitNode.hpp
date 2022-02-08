/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.hpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "nova_pure_pursuit/PurePursuit.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

#include "voltron_msgs/msg/trajectories.hpp"
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/steering_angle.hpp"

using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using Trajectory = autoware_auto_msgs::msg::Trajectory;
using namespace std::chrono_literals;


namespace Nova {
namespace PurePursuit {

constexpr auto message_frequency = 100ms; // will change

class PurePursuitNode : public rclcpp::Node {

public:

    PurePursuitNode();
    ~PurePursuitNode();

private:  
    
    // publish steering angle
    rclcpp::Publisher<voltron_msgs::msg::SteeringAngle>::SharedPtr steering_control_publisher;
    
    // subscribe to get trajectory
    rclcpp::Subscription<voltron_msgs::msg::Trajectories>::SharedPtr trajectory_subscription;

    // var
    std::unique_ptr<PurePursuit> controller;
    rclcpp::TimerBase::SharedPtr control_timer;
    voltron_msgs::msg::Trajectory trajectory;
    std::vector<TrajectoryPoint> test_trajectory;

    // functions
    void send_message();
    void update_trajectory(voltron_msgs::msg::Trajectories::SharedPtr trajectories);
    void load_test_trajectory(std::vector<TrajectoryPoint> &v);
};


}
}