/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#include <functional>
#include <string>
#include <math.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/trajectory_point.hpp"
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include "std_msgs/msg/float32.hpp"

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_components/register_node_macro.hpp>

#include <motion_common/motion_common.hpp>

#include "motion_planner/MotionPlannerNode.hpp"
#include "motion_planner/MotionPlanner.hpp"

using namespace navigator::MotionPlanner;

MotionPlannerNode::MotionPlannerNode(const rclcpp::NodeOptions &node_options) : 
    Node("motion_planner_node", node_options) ,
    tf_listener(tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
    RCLCPP_WARN(this->get_logger(), "motion planner constructor");
    //todo update names
    trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectories", 8);
    path_subscription = this->create_subscription<voltron_msgs::msg::CostedPaths>("paths", 8, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    current_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/lgsvl/gnss_odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    auto trajectory_message = voltron_msgs::msg::Trajectory();
    auto trajectory = planner->get_trajectory(ideal_path, pose);
    //maybe change trajectory vector over to the message type to avoid this
    for (size_t i = 0; i < trajectory->size(); i++) {
        auto point = voltron_msgs::msg::TrajectoryPoint();
        point.x = trajectory->at(i).x;
        point.y = trajectory->at(i).y;
        point.vx = 0;
        point.vy = 0;
        trajectory_message.points.push_back(point);
    }
    trajectory_publisher->publish(trajectory_message);
    //RCLCPP_WARN(this->get_logger(), "published trajectory");
}

void MotionPlannerNode::update_path(voltron_msgs::msg::CostedPaths::SharedPtr ptr) {
    //RCLCPP_WARN(this->get_logger(), "Got path");
    //select minimum cost path
    size_t min_index = 0;
    double min_cost = ptr->paths[min_index].safety_cost+ptr->paths[min_index].routing_cost;
    for (size_t i = 1; i < ptr->paths.size(); i++) {
        double item_cost = ptr->paths[i].safety_cost+ptr->paths[i].routing_cost;
        if (min_cost > item_cost) {
            min_cost = item_cost;
            min_index = i;
        }
    }
    ideal_path = ptr->paths[min_index];
}

void MotionPlannerNode::update_pose(const geometry_msgs::msg::PoseStamped& pose_in) {
    pose.x = pose_in.pose.position.x;
    pose.y = pose_in.pose.position.y;
    
    //velocity, heading updated in current_pose_cb
}

/**
 * @brief Callback for kinematic state subscriber
 * 
 * Transform to TrajectoryPoint and pass to PathPlanner logic.
 * Copied from autoware code in Lanelet2GlobalPlannerNode
 * 
 * @param state 
 */
void MotionPlannerNode::current_pose_cb(const nav_msgs::msg::Odometry::SharedPtr state_msg)
{
  // convert msg to geometry_msgs::msg::Pose
  current_pose.pose.position.x = state_msg->pose.pose.position.x;
  current_pose.pose.position.y = state_msg->pose.pose.position.y;
  current_pose.pose.position.z = 0.0;
  current_pose.pose.orientation = state_msg->pose.pose.orientation;
  current_pose.header = state_msg->header;
  //update velocity
  pose.xv = state_msg->twist.twist.linear.x;//.state.longitudinal_velocity_mps;
  pose.yv = state_msg->twist.twist.linear.y;//state_msg->state.lateral_velocity_mps;
  pose.heading = state_msg->twist.twist.angular.z;

  // transform to "map" frame if needed
  if (current_pose.header.frame_id != "map")
  {
    geometry_msgs::msg::PoseStamped current_pose_map = current_pose;

    if (transform_pose_to_map(current_pose, current_pose_map))
    {
      // transform ok: set current_pose to the pose in map
      current_pose = current_pose_map;
      //current_pose_init = true;
    }
    else
    {
      // transform failed, don't want to use untransformed pose
      //current_pose_init = false;
    }
  }
  else
  {
    // No transform required
    //current_pose_init = true;
  }
    update_pose(current_pose);

}

/**
 * Copied from Lanelet2GlobalPlannerNode autoware code
 * 
 * @param pose_in 
 * @param pose_out 
 * @return true 
 * @return false 
 */
bool MotionPlannerNode::transform_pose_to_map(
          const geometry_msgs::msg::PoseStamped &pose_in,
          geometry_msgs::msg::PoseStamped &pose_out)
      {
        std::string source_frame = pose_in.header.frame_id;
        // lookup transform validity
        if (!tf_buffer.canTransform("map", source_frame, tf2::TimePointZero))
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to transform Pose to map frame");
          return false;
        }

        // transform pose into map frame
        geometry_msgs::msg::TransformStamped tf_map;
        try
        {
          tf_map = tf_buffer.lookupTransform(
              "map", source_frame,
              time_utils::from_message(pose_in.header.stamp));
        }
        catch (const tf2::ExtrapolationException &)
        {
          // currently falls back to retrive newest transform available for availability,
          // Do validation of time stamp in the future
          tf_map = tf_buffer.lookupTransform("map", source_frame, tf2::TimePointZero);
        }

        // apply transform
        tf2::doTransform(pose_in, pose_out, tf_map);
        return true;
      }

RCLCPP_COMPONENTS_REGISTER_NODE(navigator::MotionPlanner::MotionPlannerNode)