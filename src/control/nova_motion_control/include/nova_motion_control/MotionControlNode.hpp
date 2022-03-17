/*
 * Package:   nova_motion_control
 * Filename:  MotionControlNode.hpp
 * Author:    Cristian Cruz, Nikhil Narvekar
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "nova_motion_control/PurePursuit.hpp"
#include "nova_motion_control/PidController.hpp"

#include "rclcpp/rclcpp.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"
#include <autoware_auto_msgs/msg/vehicle_control_command.hpp>
#include "autoware_auto_msgs/msg/trajectory.hpp"
#include <autoware_auto_msgs/msg/trajectory.hpp>

#include "voltron_msgs/msg/steering_position.hpp"
#include "voltron_msgs/msg/peddle_position.hpp"



using Trajectory = autoware_auto_msgs::msg::Trajectory;
using TrajectoryPoint = autoware_auto_msgs::msg::TrajectoryPoint;
using SteeringPosition = voltron_msgs::msg::SteeringPosition;
using PeddlePosition = voltron_msgs::msg::PeddlePosition;
using Odometry = nav_msgs::msg::Odometry;

using Marker = visualization_msgs::msg::Marker;
using MarkerArray = visualization_msgs::msg::MarkerArray;

using namespace std::chrono_literals;

namespace Nova {
namespace MotionControl {

constexpr auto message_frequency = 100ms; // will change

class MotionControlNode : public rclcpp::Node {

public:

    MotionControlNode();
    ~MotionControlNode();
    

private:  
    // var
    std::unique_ptr<Nova::PurePursuit::PurePursuit> steering_controller;
    std::unique_ptr<Nova::PidController::PidController> speed_controller;

    rclcpp::Publisher<MarkerArray>::SharedPtr marker_array_publisher;
    rclcpp::TimerBase::SharedPtr control_timer;

    rclcpp::Subscription<Trajectory>::SharedPtr trajectory_subscription;
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;
    rclcpp::Publisher<SteeringPosition>::SharedPtr steering_control_publisher;
    rclcpp::Publisher<PeddlePosition>::SharedPtr throttle_control_publisher;
    rclcpp::Publisher<PeddlePosition>::SharedPtr brake_control_publisher;

    Trajectory trajectory;
    TrajectoryPoint current_position;
    float current_speed;

    typedef std::chrono::steady_clock clock_t;
    clock_t clock;
    clock_t::time_point last_update_time;

    // functions
    void send_message();
    void update_trajectory(Trajectory::SharedPtr ptr);
    void update_odometry(Odometry::SharedPtr ptr);

    void visualize_markers(std::string frame_id, rclcpp::Time time);
    float distance(TrajectoryPoint p1, TrajectoryPoint P2);

    size_t find_closest_point();
    size_t find_next_waypoint(float lookahead_distance, TrajectoryPoint& current_position);

    void trim_trajectory(size_t current_point_idx);
    bool compute_lookahead_point();
    void compute_target_speed();
    float get_steering_angle();
};


}
}