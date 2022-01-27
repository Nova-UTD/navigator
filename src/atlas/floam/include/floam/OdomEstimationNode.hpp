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
#include <sensor_msgs/msg/point_cloud2.hpp>
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
#include "lidar.h"
#include "odomEstimationClass.h"

using sensor_msgs::msg::PointCloud2;
using nav_msgs::msg::Odometry;
using namespace std::chrono_literals;

namespace Nova {
namespace Atlas {

    class OdomEstimationNode : public rclcpp::Node {
    public:
    OdomEstimationNode();

    private:
        void setLidarParams();
        void estimateOdometry();
        void velodyneSurfHandler(const PointCloud2::SharedPtr &laserCloudMsg);
        void velodyneEdgeHandler(const PointCloud2::SharedPtr &laserCloudMsg);

        rclcpp::Publisher<Odometry>::SharedPtr laser_odom_pub_;
        rclcpp::Subscription<PointCloud2>::SharedPtr edge_pcd_sub_, surf_pcd_sub_;
        std::queue<PointCloud2::SharedPtr> pointCloudEdgeBuf;
        std::queue<PointCloud2::SharedPtr> pointCloudSurfBuf;

        OdomEstimationClass odom_estimator_;
        std::mutex mutex_lock_;

        lidar::Lidar lidar_param_;
        bool is_odom_inited = false;
        double total_time =0;
        int total_frame=0;
    
    };

} // Speedometer
} // Nova
