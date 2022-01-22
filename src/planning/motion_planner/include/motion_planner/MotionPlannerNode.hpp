/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include <chrono> // Time literals
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/trajectory.hpp"
#include "std_msgs/msg/float32.hpp"

#include "motion_planner/MotionPlanner.hpp"

using namespace std::chrono_literals;

namespace navigator {
namespace MotionPlanner {

// How often to publish the new trajectory message
constexpr auto message_frequency = 20ms;

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode();

private:
    void send_message();
    //temp pointer types
    void update_path(std_msgs::msg::Float32::SharedPtr ptr);
    void update_perception(std_msgs::msg::Float32::SharedPtr ptr);

    rclcpp::Publisher<voltron_msgs::msg::Trajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr path_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr perception_subscription;
    rclcpp::TimerBase::SharedPtr control_timer;

    MotionPlanner planner;
    std::shared_ptr<std::vector<IdealPoint>> ideal_path;
    CarPose pose;
};
}
}
