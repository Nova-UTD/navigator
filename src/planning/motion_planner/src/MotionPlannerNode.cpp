/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

//NOTE: LOOK IN HEADER FILE FOR COMMENTS ON FUNCTION DEFINITIONS

//#include <math.h>

#include "rclcpp/rclcpp.hpp"

#include "motion_planner/MotionPlannerNode.hpp"

using namespace navigator::motion_planner;

MotionPlannerNode::MotionPlannerNode() : Node("motion_planner_node")
{
    trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectory", 8);
    path_subscription = this->create_subscription<voltron_msgs::msg::FinalPath>("/planning/paths", 10, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    //odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    //current_pose_subscription = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    //steering_angle_subscription = this->create_subscription<voltron_msgs::msg::SteeringPosition>("/can/steering_angle", 8, bind(&MotionPlannerNode::update_steering_angle, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
    //planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    if (ideal_path == nullptr) {
        RCLCPP_WARN(this->get_logger(), "motion planner has no input path, skipping...");
        return;
    }
    RCLCPP_WARN(this->get_logger(), "ideal path has %d points", ideal_path->points.size());
    auto tmp = voltron_msgs::msg::Trajectory();
    for (size_t i = 0; i < ideal_path->points.size(); i++) {
      auto t = voltron_msgs::msg::TrajectoryPoint();
      auto p = ideal_path->points[i];
      t.x = p.x;
      t.y = p.y;
      t.vx = ideal_path->speeds[i];
      tmp.points.push_back(t);
    }
    trajectory_publisher->publish(tmp);
    return;
}

void MotionPlannerNode::update_path(voltron_msgs::msg::FinalPath::SharedPtr ptr) {
    ideal_path = ptr;
}

/*void MotionPlannerNode::update_steering_angle(voltron_msgs::msg::SteeringPosition::SharedPtr ptr) {
  steering_angle = ptr->data; //radians
}

void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  pose.heading = quat_to_heading(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.xv = msg->twist.twist.linear.x;
  pose.yv = msg->twist.twist.linear.y;
}

//radians
double MotionPlannerNode::quat_to_heading(double x, double y, double z, double w) {
  //z component of euler angles
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(t3, t4) + M_PI;
}*/