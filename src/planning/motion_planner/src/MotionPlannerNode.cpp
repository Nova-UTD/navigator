/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

//NOTE: LOOK IN HEADER FILE FOR COMMENTS ON FUNCTION DEFINITIONS

#include <functional>
#include <string>
#include <math.h>
#include <sstream>

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
    trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectories", 8);
    trajectory_viz_publisher = this->create_publisher<voltron_msgs::msg::Trajectories>("outgoing_trajectories_viz", 8);
    path_subscription = this->create_subscription<voltron_msgs::msg::CostedPaths>("paths", 8, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    //current_pose_subscription = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    steering_angle_subscription = this->create_subscription<std_msgs::msg::Float32>("real_steering_angle", 8, bind(&MotionPlannerNode::update_steering_angle, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
    planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    auto trajectories = voltron_msgs::msg::Trajectories();
    //get candidate costed trajectories from the motion planner class
    std::vector<CarPose> colliders;
    auto candidates = planner->get_trajectory(ideal_path, pose, colliders);

    //find min cost path
    double min_cost = candidates->at(0).cost;
    size_t min_index = 0;
    for (size_t i = 0; i < candidates->size(); i++) {
        double cost = candidates->at(i).cost;
        if (cost < min_cost) {
            min_cost = cost;
            min_index = i;
        }
    }

    //build message with trajectories
    //currently sending all for visualization purposes
    //should refactor this to only send one, and do the visualization in this node
    for (size_t t = 0; t < candidates->size(); t++) {
      auto trajectory_message = voltron_msgs::msg::Trajectory();
      auto trajectory = candidates->at(t);
      for (size_t i = 0; i < trajectory.points->size(); i++) {
        auto point = voltron_msgs::msg::TrajectoryPoint();
        point.x = trajectory.points->at(i).x;
        point.y = trajectory.points->at(i).y;
        point.vx = trajectory.points->at(i).vx;
        point.vy = trajectory.points->at(i).vy;
        trajectory_message.points.push_back(point);
      }
      trajectory_message.id = t;
      trajectory_message.selected = (t == min_index) ? 1 : 0;
      trajectories.trajectories.push_back(trajectory_message);
    }
    trajectory_viz_publisher->publish(trajectories);
    trajectory_publisher->publish(trajectories.trajectories[min_index]);
}

void MotionPlannerNode::update_path(voltron_msgs::msg::CostedPaths::SharedPtr ptr) {
    if (ptr->paths.size() == 0) {
      return; //if there is no path, continue using the old one
      //should probably raise an alert, but idk if that's this node's responsibility
    }
    //select min cost path
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

void MotionPlannerNode::update_steering_angle(std_msgs::msg::Float32::SharedPtr ptr) {
  steering_angle = ptr->data; //radians
}

void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  pose.heading = 2*asin(msg->pose.pose.orientation.z);
  current_pose.pose.orientation = msg->pose.pose.orientation;
  current_pose.pose.position = msg->pose.pose.position;
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.xv = msg->twist.twist.linear.x;
  pose.yv = msg->twist.twist.linear.y;
}

/**
 * @brief Callback for kinematic state subscriber
 * 
 * Transform to TrajectoryPoint and pass to PathPlanner logic.
 * Copied from autoware code in Lanelet2GlobalPlannerNode
 * 
 * May be removed?
 * 
 * @param state 
 */
void MotionPlannerNode::current_pose_cb(const VehicleKinematicState::SharedPtr state_msg)
{
  // TODO: validate header

  // convert msg to geometry_msgs::msg::Pose
  current_pose.pose.position.x = state_msg->state.x;
  current_pose.pose.position.y = state_msg->state.y;
  current_pose.pose.position.z = 0.0;
  current_pose.pose.orientation =
      motion::motion_common::to_quat<geometry_msgs::msg::Quaternion>(
          state_msg->state.heading);
  current_pose.header = state_msg->header;

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
  pose.x = current_pose.pose.position.x;
  pose.y = current_pose.pose.position.y;
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
        RCLCPP_WARN(this->get_logger(), "transform pose to map");
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