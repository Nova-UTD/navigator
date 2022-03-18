/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

/*
    This node is the heart of the motion planner. It gathers information
    from the path planner (and, in the future, the behavior planner and perception)
    to plan the cars immediate trajectory.

    Trajectory generation and costing is done in the MotionPlanner class.
    
    This node sends trajectory information in the /planning/outgoing_trajectories message
        (Look at Trajectories.msg for format)
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
#include "voltron_msgs/msg/trajectories.hpp"
#include "voltron_msgs/msg/final_path.hpp"
#include "voltron_msgs/msg/steering_position.hpp"
#include "std_msgs/msg/float32.hpp"

#include "motion_planner/MotionPlanner.hpp"
#include "motion_planner/CarPose.hpp"

using namespace std::chrono_literals;
using namespace autoware_auto_msgs::msg;

namespace navigator {
namespace MotionPlanner {

// How often to publish the new trajectory message
constexpr auto message_frequency = 100ms;

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode(const rclcpp::NodeOptions &node_options);

private:
    //calls MotionPlanner to get the trajectories, selects one, and sends a message containing all trajectories
    //in the future, this will only send the selected trajectory. but sending all is useful for testing
    void send_message();
    //subscription to behavior planner for input ideal path.
    void update_path(voltron_msgs::msg::FinalPath::SharedPtr ptr);
    //does nothing at the moment
    void update_perception(std_msgs::msg::Float32::SharedPtr ptr);
    //gets the current x,y position of the car
    void current_pose_cb(const VehicleKinematicState::SharedPtr ptr);
    //gets the current heading of the car
    void odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg);
    //helper function for current_pose_cb
    bool transform_pose_to_map(const geometry_msgs::msg::PoseStamped &pose_in,
          geometry_msgs::msg::PoseStamped &pose_out);
    void update_steering_angle(voltron_msgs::msg::SteeringPosition::SharedPtr ptr);

    rclcpp::Publisher<voltron_msgs::msg::Trajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<voltron_msgs::msg::Trajectories>::SharedPtr trajectory_viz_publisher;
    rclcpp::Subscription<voltron_msgs::msg::FinalPath>::SharedPtr path_subscription;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr perception_subscription;
    rclcpp::Subscription<VehicleKinematicState>::SharedPtr current_pose_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomtery_pose_subscription;
    rclcpp::Subscription<voltron_msgs::msg::SteeringPosition>::SharedPtr steering_angle_subscription; //radians
    rclcpp::TimerBase::SharedPtr control_timer;

    std::shared_ptr<MotionPlanner> planner;
    voltron_msgs::msg::FinalPath::SharedPtr ideal_path;
    geometry_msgs::msg::PoseStamped current_pose;
    CarPose pose;
    float steering_angle; //radians
    
    // Needed for post to map transform
    tf2::BufferCore tf_buffer;
    tf2_ros::TransformListener tf_listener;
};
}
}