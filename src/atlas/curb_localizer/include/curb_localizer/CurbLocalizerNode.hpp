/*
 * Package:   curb_localizer
 * Filename:  CurbLocalizerNode.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

# pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>
#include <opendrive_utils/OpenDriveUtils.hpp>

#include "Lanes.h"
#include "Road.h"
#include "LaneSection.h"

namespace navigator
{
namespace curb_localizer
{

class CurbLocalizerNode : public rclcpp::Node
{
    public:
        CurbLocalizerNode();

    private:
        double dist_to_curb = 1;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr curb_dist_sub;
        void curb_dist_callback(const std_msgs::msg::Float32::SharedPtr msg){
            dist_to_curb = std::max(0.1f, msg->data);
        }

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in_sub;
        void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_pub;

        std::string map_file_path;
        opendrive::OpenDriveMapPtr map;

        std::shared_ptr<nav_msgs::msg::Odometry> odom_in;
        double bias_y = 0;
        double bias_x = 0;

        void publish_odom();

};
}
} // namespace navigator
