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

#include "rclcpp/rclcpp.hpp"

#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <autoware_auto_msgs/msg/trajectory.hpp>
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>

#include "voltron_msgs/msg/steering_angle.hpp"

using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using SteeringAngle = voltron_msgs::msg::SteeringAngle;


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
    rclcpp::Publisher<SteeringAngle>::SharedPtr steering_control_publisher;
    
    // subscribe to get trajectory
    rclcpp::Subscription<Trajectory>::SharedPtr trajectory_subscription;

    // var
    std::unique_ptr<PurePursuit> controller;
    rclcpp::TimerBase::SharedPtr control_timer;
    Trajectory trajectory;

    // functions
    void send_message();
    void update_trajectory(Trajectory::SharedPtr trajectory);
    void load_test_trajectory(std::vector<autoware_auto_msgs::msg::TrajectoryPoint> &v);
    TrajectoryPoint compute_closest_point(TrajectoryPoint current_point);
};


}
}