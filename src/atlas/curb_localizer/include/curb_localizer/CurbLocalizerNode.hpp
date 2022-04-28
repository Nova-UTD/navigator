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
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opendrive_utils/OpenDriveUtils.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace navigator
{
namespace curb_localizer
{

class CurbLocalizerNode : public rclcpp::Node
{
    public:
        CurbLocalizerNode();

    private:
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_curb_points_sub;
        void left_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_curb_points_sub;
        void right_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in_sub;
        void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out;

        std::string map_file_path;
        opendrive::OpenDriveMapPtr map;

        pcl::PointCloud<pcl::PointXYZ> left_curb_points;
        pcl::PointCloud<pcl::PointXYZ> right_curb_points;

        std::shared_ptr<nav_msgs::msg::Odometry> odom_in;

        void transform_points_to_odom(const pcl::PointCloud<pcl::PointXYZ> &in_cloud,
            const nav_msgs::msg::Odometry &odom,
            pcl::PointCloud<pcl::PointXYZ> &out_cloud);


};
}
} // namespace navigator
