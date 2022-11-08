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
#include "voltron_msgs/msg/DOGMa.hpp"
#include "voltron_msgs/msg/GPSDiagnostic.hpp"

using namespace std::chrono_literals;

namespace navigator {
namespace cost_map_maker {

// How often to publish the new trajectory message
constexpr auto message_frequency = 250ms;

class CostMapMakerNode : public rclcpp::Node {
public:
    // Constructor
    CostMapMakerNode();

    // Functions

private:

    // Send final cost map to subscribers
    void send_message();
    // Subscription to *INSERT PUBLISHER* for input Dynamic Occupancy Grid
    void update_DOGMa(voltron_msgs::msg::DOGMa::SharedPtr ptr);
    // Subscription to *INSERT PUBLISHER* for input waypoints
    void update_waypoints(voltron_msgs::msg::ZoneArray::SharedPtr ptr);
    // Subscription to *INSERT PUBLISHER* for the current heading of the car
    void odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Cost map Publisher
    rclcpp::Publisher<voltron_msgs::msg::DOGMa>::SharedPtr cost_map_publisher;
    // Dynamic Occupancy Grid Subscriber
    rclcpp::Subscription<voltron_msgs::msg::DOGMa>::SharedPtr DOGMa_subscription;
    /* TODO Add information from global waypoints */
    rclcpp::Subscription<voltron_msgs::msg::ZoneArray>::SharedPtr waypoint_subscription;

    // Odometry Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomtery_pose_subscription;
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer;


    voltron_msgs::msg::DOGMa::SharedPtr DOGMa;
    /* TODO Add information from global waypoints */
    voltron_msgs::msg::ZoneArray::SharedPtr waypoints;
    nav_msgs::msg::Odometry::SharedPtr odometry;
};
}
}
