/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

/*
    Currently, this node gets an Evidential Grid Map input from perception and creates a cost map accordingly to ensure safety and good driving practices.
    It does so by giving collisions an extremely high costs and making spaces closer to the next waypoint less costly. *TO BE CONTINUED*
*/

#pragma once

#include <chrono> // Time literals
#include <vector>
#include <string>

// libOpenDRIVE includes
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"
#include "Geometries/Line.h"
#include "RefLine.h"
#include "opendrive_utils/OpenDriveUtils.hpp"

// Message includes
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include "nova_msgs/msg/egma.hpp"
#include "nova_msgs/msg/evidential_grid.hpp"
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    // Subscription to *INSERT PUBLISHER* for input Evidential Grid Map
    void update_egma(nova_msgs::msg::Egma::SharedPtr ptr);
    // Subscription to *INSERT PUBLISHER* for input waypoints
    void update_waypoints(geometry_msgs::msg::Vector3::SharedPtr ptr);
    // Subscription to *INSERT PUBLISHER* for the current heading of the car
    void odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Cost Map Publisher
    rclcpp::Publisher<nova_msgs::msg::Egma>::SharedPtr cost_map_publisher;
    // Evidential Grid Map Subscriber
    rclcpp::Subscription<nova_msgs::msg::Egma>::SharedPtr egma_subscription;
    /* TODO Add information from global waypoints */
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr waypoint_subscription;

    // Odometry Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomtery_pose_subscription;
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer;


    // EGMa from prediction
    nova_msgs::msg::Egma::SharedPtr egma;

    /* TODO Add information from global waypoints */
    navigator::opendrive::OpenDriveMapPtr carla_map;
    geometry_msgs::msg::Vector3::SharedPtr waypoints;
    nav_msgs::msg::Odometry::SharedPtr odometry;
};
}
}
