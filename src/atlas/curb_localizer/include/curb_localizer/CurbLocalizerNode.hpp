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

#include <Eigen/Dense>

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
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr left_curb_points_sub;
        void left_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr right_curb_points_sub;
        void right_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_in_sub;
        void odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_out_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_pub2;


        double look_distance; // meters to look ahead and behind the car for the curb

        std::string map_file_path;
        opendrive::OpenDriveMapPtr map;

        pcl::PointCloud<pcl::PointXYZ>::Ptr left_curb_points;
        pcl::PointCloud<pcl::PointXYZ>::Ptr right_curb_points;

        std::shared_ptr<nav_msgs::msg::Odometry> odom_in;
        Eigen::Vector3d last_offset;

        void publish_odom();

        static void transform_points_to_odom(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
            const nav_msgs::msg::Odometry &odom,
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
        // Projects the cloud onto the XY plane.
        static void flatten_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
        // Finds the translation required to map the estimate (current lidar with odom) to the
        // known truth (curb on map data)
        static Eigen::Vector3d find_translation(const pcl::PointCloud<pcl::PointXYZ>::Ptr truth,
            const pcl::PointCloud<pcl::PointXYZ>::Ptr estimate);

};
}
} // namespace navigator
