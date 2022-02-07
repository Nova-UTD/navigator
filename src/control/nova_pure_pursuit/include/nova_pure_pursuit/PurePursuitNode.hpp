/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.hpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "voltron_msgs/msg/trajectories.hpp"
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/steering_angle.hpp"

#include "nova_pure_pursuit/PurePursuit.hpp"

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

    // functions
    void send_message();
    void update_trajectory(voltron_msgs::msg::Trajectories::SharedPtr trajectories);
};


}
}