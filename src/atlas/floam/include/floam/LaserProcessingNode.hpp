/*
 * Package:   epas_translator
 * Filename:  ControllerNode.hpp
 * Author:    Joshua Williams
 * Email:     joshmackwilliams@protonmail.com
 * Copyright: 2021, Voltron UTD
 * License:   MIT License
 */

#pragma once

//c++ lib
#include <cmath>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <chrono>

//ros lib
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
// #include <tf2_ros/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>

//pcl lib
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include <floam/lidar.h>
#include <floam/laserProcessingClass.h>

using sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;

namespace Nova {
namespace Atlas {

    class LaserProcessingNode : public rclcpp::Node {
    public:
        LaserProcessingNode();

    private:
        void processCloud();
        void velodyneHandler(const PointCloud2::SharedPtr &laserCloudMsg);
        
        lidar::Lidar lidar_param_;
        LaserProcessingClass laser_processor_;
        std::mutex mutex_lock_;
        std::queue<PointCloud2::SharedPtr> pcd_queue_;

        rclcpp::Publisher<PointCloud2>::SharedPtr edge_pcd_pub_;
        rclcpp::Publisher<PointCloud2>::SharedPtr surf_pcd_pub_;
        rclcpp::Publisher<PointCloud2>::SharedPtr filtered_pcd_pub_;
        rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub_;
    };

} // Speedometer
} // Nova
