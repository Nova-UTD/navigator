/*
 * Package:   MotionPlannerVisuals
 * Filename:  MotionPlannerVisualsNode.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/trajectory_point.hpp"
#include "std_msgs/msg/float32.hpp"
#include "visualization_msgs/msg/marker.hpp"

#include "motion_planner_visuals/MotionPlannerVisualsNode.hpp"

using namespace navigator::MotionPlannerVisuals;

MotionPlannerVisualsNode::MotionPlannerVisualsNode() : Node("MotionPlannerVisuals") {
    //todo update names
    this->visuals_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/MotionPlannerVisuals/trajectory", 8);
    this->trajectory_subscription = this->create_subscription<voltron_msgs::msg::Trajectory>("outgoing_trajectories", 8, bind(&MotionPlannerVisualsNode::send_message, this, std::placeholders::_1));
}

void MotionPlannerVisualsNode::send_message(const voltron_msgs::msg::Trajectory::SharedPtr msg) {
    auto markers = visualization_msgs::msg::MarkerArray();

    for (size_t i = 0; i < msg->points.size(); i++) {
        auto point = msg->points.at(i);
        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(250ms);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.id = i;

        marker.scale.x = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

        marker.pose.position.x = point.x;
        marker.pose.position.z = point.y;

        markers.markers.push_back(marker);

    }
    
    this->visuals_publisher->publish(markers);
}