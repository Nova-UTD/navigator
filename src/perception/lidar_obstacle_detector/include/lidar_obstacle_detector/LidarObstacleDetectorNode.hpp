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
#include "voltron_msgs/msg/zone_array.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace navigator
{
namespace lidar_obstacle_detector
{
using sensor_msgs::msg::PointCloud2;
using voltron_msgs::msg::ZoneArray;
using voltron_msgs::msg::Zone;

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
        double zone_padding; // pad zones by this amount 

        // Stores nearest distances to obstacles at each angle
        std::vector<double> nearest_obstacles;
        int to_nearest_obstacle_index(double angle);

        // Subscribe to lidar
        rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub;
        void lidar_callback(const PointCloud2::SharedPtr msg);

        // publish obstacle zones
        rclcpp::Publisher<ZoneArray>::SharedPtr zone_pub;
        void publish_zones();
        rclcpp::TimerBase::SharedPtr zone_timer;

        // Transform tree
        std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
        void update_tf();
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped currentTf;
        void transform_zone(Zone& zone);

};
} // namespace lidar_obstacle_detector
} // namespace navigator