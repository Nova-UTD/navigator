/*
 * Package:   motion_planner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <algorithm> // std::clamp
#include <cstdint>   // Fixed-width integers
#include <functional>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/Trajectory.hpp"
#include "voltron_msgs/msg/TrajectoryPoint.hpp"
#include "std_msgs/msg/float32.hpp"

#include "motion_planner/MotionPlannerNode.hpp"

using namespace Nova::motion_planner;

MotionPlannerNode::MotionPlannerNode() : Node("motion_planner") {
    //todo update names
    this->trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectories", 8);
    this->power_subscription = this->create_subscription<std_msgs::msg::Float32>("steering_power", 8, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    this->control_timer = this->create_wall_timer(message_frequency, bind(&ControllerNode::send_message, this));
}

void MotionPlannerNode::send_message() {
    auto trajectory_message = voltron_msgs::msg::Trajectory();
    auto trajectory = planer.get_trajectory(ideal_path, pose);
    //maybe change trajectory vector over to the message type to avoid this
    for (size_t i = 0; i < trajectory->size(); i++) {
        auto point = voltron_msgs::msg::TrajectoryPoint();
        point.x = trajectory->get(i).x;
        point.y = trajectory->get(i).y;
        point.vx = 0;
        point.vy = 0;
        trajectory_message.points.push_back(point);
    }
    this->trajectory_publisher->publish(trajectory_message);
}