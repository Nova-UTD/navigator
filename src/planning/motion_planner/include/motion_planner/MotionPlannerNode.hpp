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
//#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/costed_paths.hpp"
#include "std_msgs/msg/float32.hpp"

#include "motion_planner/MotionPlanner.hpp"
#include "motion_planner/CarPose.hpp"

using namespace std::chrono_literals;
using namespace autoware_auto_msgs::msg;

namespace navigator {
namespace MotionPlanner {

// How often to publish the new trajectory message
constexpr auto message_frequency = 20ms;

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode(const rclcpp::NodeOptions &node_options);

private:
    void send_message();
    void update_path(voltron_msgs::msg::CostedPaths::SharedPtr ptr);
    void update_perception(std_msgs::msg::Float32::SharedPtr ptr);
    void current_pose_cb(const nav_msgs::msg::Odometry::SharedPtr ptr);
    bool transform_pose_to_map(const geometry_msgs::msg::PoseStamped &pose_in,
          geometry_msgs::msg::PoseStamped &pose_out);
    void update_pose(const geometry_msgs::msg::PoseStamped& pose);

    rclcpp::Publisher<voltron_msgs::msg::Trajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<voltron_msgs::msg::CostedPaths>::SharedPtr path_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr perception_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_subscription;
    rclcpp::TimerBase::SharedPtr control_timer;

    std::shared_ptr<MotionPlanner> planner;
    voltron_msgs::msg::CostedPath ideal_path;
    geometry_msgs::msg::PoseStamped current_pose;
    CarPose pose;
    
    // Needed for post to map transform
    tf2::BufferCore tf_buffer;
    tf2_ros::TransformListener tf_listener;
};
}
}
