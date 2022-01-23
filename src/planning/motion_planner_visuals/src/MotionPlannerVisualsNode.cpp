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
#include <rclcpp_components/register_node_macro.hpp>

#include "motion_planner_visuals/MotionPlannerVisualsNode.hpp"

using namespace navigator::MotionPlannerVisuals;

MotionPlannerVisualsNode::MotionPlannerVisualsNode(const rclcpp::NodeOptions &node_options) : 
    Node("motion_planner_visuals_node", node_options)
{
    this->visuals_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("motion_planner_visuals", 8);
    this->trajectory_subscription = this->create_subscription<voltron_msgs::msg::Trajectory>("outgoing_trajectories", 8, bind(&MotionPlannerVisualsNode::send_message, this, std::placeholders::_1));
}

void MotionPlannerVisualsNode::send_message(const voltron_msgs::msg::Trajectory::SharedPtr msg) {
    auto markers = visualization_msgs::msg::MarkerArray();

        auto marker = visualization_msgs::msg::Marker();
        marker.header.stamp = this->now();
        marker.header.frame_id = "map";
        marker.ns = "motion_planner_visuals";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(250ms);
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.id = 0;
        marker.scale.x = 0.2;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;

    for (size_t i = 0; i < msg->points.size(); i++) {
        auto point = msg->points.at(i);
        auto message_point = geometry_msgs::msg::Point();
        message_point.x = point.x;
        message_point.y = point.y;
        message_point.z = 0;
        marker.points.push_back(message_point);
    }
    markers.markers.push_back(marker);
    this->visuals_publisher->publish(markers);
}

RCLCPP_COMPONENTS_REGISTER_NODE(navigator::MotionPlannerVisuals::MotionPlannerVisualsNode)