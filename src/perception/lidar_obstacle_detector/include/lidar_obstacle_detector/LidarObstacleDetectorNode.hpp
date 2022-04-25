/*
 * Package:   lidar_obstacle_detector
 * Filename:  LidarObstacleDetectorNode.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"



namespace navigator
{
namespace lidar_obstacle_detector
{
using sensor_msgs::msg::PointCloud2;

class LidarObstacleDetectorNode : public rclcpp::Node
{
    public:
        LidarObstacleDetectorNode();

    private:
        // parameters
        double min_obstacle_height; // set the floor in z-coordinates to avoid registering the ground
        double incline_rate;    // raise the floor conically at this rate per meter
        double max_obstacle_height; // set the ceiling to avoid registering trees
        double fov_angle; // field of view to look for obstacles in
        int fov_segments; // segment the field of view into this many segments
        double angle_resolution; // (fov_angle / fov_segments)

        // Stores nearest distances to obstacles at each angle
        std::vector<double> nearest_obstacles;
        int to_nearest_obstacle_index(double angle);

        // Subscribe to lidar
        rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub;
        void lidar_callback(const PointCloud2::SharedPtr msg);

};
} // namespace lidar_obstacle_detector
} // namespace navigator