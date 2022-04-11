/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

/*
    Currently, this node republishes data from the path publisher. In the future,
    It will take in zones from the traffic planner and obstacle zoner
    and assign speeds to the path.
*/

#pragma once

#include <chrono> // Time literals
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "voltron_msgs/msg/trajectory.hpp"
#include "voltron_msgs/msg/final_path.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "voltron_msgs/msg/zone_array.hpp"

using namespace std::chrono_literals;

namespace navigator {
namespace motion_planner {

// How often to publish the new trajectory message
constexpr auto message_frequency = 250ms;

class MotionPlannerNode : public rclcpp::Node {
public:
    MotionPlannerNode();

    void smooth(voltron_msgs::msg::Trajectory& trajectory, double max_accel, double max_decel);
    void limit_to_zones(voltron_msgs::msg::Trajectory& trajectory, voltron_msgs::msg::ZoneArray& zones);
    void limit_to_curvature(voltron_msgs::msg::Trajectory& trajectory, double max_acceleration, double curvature_interval = 2.0);
    voltron_msgs::msg::Trajectory build_trajectory(voltron_msgs::msg::FinalPath::SharedPtr path, size_t horizon);

private:
    //how many points on path to actually consider
    static constexpr size_t horizon = 100;
    //calls MotionPlanner to get the trajectories, selects one, and sends a message containing all trajectories
    //in the future, this will only send the selected trajectory. but sending all is useful for testing
    void send_message();
    //subscription to behavior planner for input ideal path.
    void update_path(voltron_msgs::msg::FinalPath::SharedPtr ptr);
    // subscription to behavior planner for input zones
    void update_zones(voltron_msgs::msg::ZoneArray::SharedPtr ptr);

    //gets the current heading of the car
    void odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

    rclcpp::Publisher<voltron_msgs::msg::Trajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<voltron_msgs::msg::FinalPath>::SharedPtr path_subscription;
    rclcpp::Subscription<voltron_msgs::msg::ZoneArray>::SharedPtr zone_subscription;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomtery_pose_subscription;

    rclcpp::TimerBase::SharedPtr control_timer;

    voltron_msgs::msg::FinalPath::SharedPtr ideal_path;
    voltron_msgs::msg::ZoneArray::SharedPtr zones;
    nav_msgs::msg::Odometry::SharedPtr odometry;
};
}
}
