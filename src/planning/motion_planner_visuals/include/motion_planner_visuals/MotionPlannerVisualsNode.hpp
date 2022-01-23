/*
 * Package:   motion_planner_visuals
 * Filename:  MotionPlannerVisualsNode.hpp
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
#include "visualization_msgs/msg/marker_array.hpp"

using namespace std::chrono_literals;

namespace navigator {
namespace MotionPlannerVisuals {

class MotionPlannerVisualsNode : public rclcpp::Node {
public:
    MotionPlannerVisualsNode(const rclcpp::NodeOptions &node_options);

private:
    void send_message(const voltron_msgs::msg::Trajectory::SharedPtr msg);

    rclcpp::Subscription<voltron_msgs::msg::Trajectory>::SharedPtr trajectory_subscription;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visuals_publisher;
};
}
}
