/*
 * Package:   lidar_obstacle_detector
 * Filename:  LidarObstacleDetectorNode.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <math.h>

#include "lidar_obstacle_detector/LidarObstacleDetectorNode.hpp"

using namespace navigator::lidar_obstacle_detector;

/**
 * @brief "arctan"
 * 
 * Operates assuming -pi/2 <= x <= pi/2,
 * and returns in the range -pi/2 <= y <= pi/2.
 * 
 * A horrendus approxmation, which gets 
 * more accurate as x gets closer to 0.
 * 
 * @param tanval 
 * @return double 
 */
double fast_arctan(double tanval){
    return tanval;
}

LidarObstacleDetectorNode::LidarObstacleDetectorNode() : rclcpp::Node("lidar_obstacle_detector"){
    this->declare_parameter("min_obstacle_height", 0.5);
    this->declare_parameter("incline_rate", 0.18);
    this->declare_parameter("max_obstacle_height", 4.0);
    this->declare_parameter("fov_angle", 2*M_PI/3);
    this->declare_parameter("fov_segments", 30);

    // Read parameters
    this->get_parameter("min_obstacle_height", min_obstacle_height);
    this->get_parameter("incline_rate", incline_rate);
    this->get_parameter("max_obstacle_height", max_obstacle_height);
    this->get_parameter("fov_angle", fov_angle);
    if(fov_angle >= M_PI){
        RCLCPP_ERROR(this->get_logger(), "fov_angle must be less than pi");
        fov_angle = 3*M_PI/4;
    }
    this->get_parameter("fov_segments", fov_segments);
    this->angle_resolution = fov_angle / fov_segments;

    // Initialize nearest_obstacles vector to infinity
    nearest_obstacles = std::vector<double>(fov_segments, std::numeric_limits<double>::infinity());

    // Subscribe to lidar
    lidar_sub = this->create_subscription<PointCloud2>("lidar_points", 10, std::bind(&LidarObstacleDetectorNode::lidar_callback, this, std::placeholders::_1));
}

int LidarObstacleDetectorNode::to_nearest_obstacle_index(double angle){
    return int((angle + fov_angle/2) / angle_resolution);
}

void LidarObstacleDetectorNode::lidar_callback(const PointCloud2::SharedPtr msg){
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud;
    pcl::fromPCLPointCloud2(pcl_cloud, *lidar_cloud);

    nearest_obstacles = std::vector<double>(fov_segments, std::numeric_limits<double>::infinity());

    // TODO: do we need to transform based on frame?

    for(auto p_it= lidar_cloud->points.begin(); p_it != lidar_cloud->points.end(); p_it++){
        pcl::PointXYZ p = *p_it;
        // X is forward
        if(p.x >0 && p.z < max_obstacle_height){
            double r = sqrt(p.x*p.x + p.y*p.y);

            double min_z = this->min_obstacle_height + r*this->incline_rate;
            if(p.z < min_z) continue;

            double angle = fast_arctan(p.y/p.x);
            if(angle < -fov_angle/2 || angle > fov_angle/2) continue;

            int cell = to_nearest_obstacle_index(angle);
            nearest_obstacles[cell] = std::min(nearest_obstacles[cell], r);
        }
    }
}