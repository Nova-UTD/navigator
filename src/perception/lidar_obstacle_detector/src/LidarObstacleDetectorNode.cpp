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

#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>

#include "lidar_obstacle_detector/LidarObstacleDetectorNode.hpp"

using namespace navigator::lidar_obstacle_detector;

using sensor_msgs::msg::PointCloud2;
using voltron_msgs::msg::ZoneArray;
using voltron_msgs::msg::Zone;
using geometry_msgs::msg::Point32;


LidarObstacleDetectorNode::LidarObstacleDetectorNode() : rclcpp::Node("lidar_obstacle_detector"){
    this->declare_parameter("min_obstacle_height", 0.5);
    this->declare_parameter("incline_rate", 0.18);
    this->declare_parameter("max_obstacle_height", 4.0);
    this->declare_parameter("fov_angle", 2*M_PI/3);
    this->declare_parameter("fov_segments", 30);
    this->declare_parameter("zone_padding", 1.5);

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
    this->get_parameter("zone_padding", zone_padding);

    // Initialize nearest_obstacles vector to infinity
    nearest_obstacles = std::vector<double>(fov_segments, std::numeric_limits<double>::infinity());

    // Subscribe to lidar
    lidar_sub = this->create_subscription<PointCloud2>("lidar_points", 10, std::bind(&LidarObstacleDetectorNode::lidar_callback, this, std::placeholders::_1));

    // Transform tree
    this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

            double angle = atan(p.y/p.x);
            if(angle < -fov_angle/2 || angle > fov_angle/2) continue;

            int cell = to_nearest_obstacle_index(angle);
            nearest_obstacles[cell] = std::min(nearest_obstacles[cell], r);
        }
    }
}

void LidarObstacleDetectorNode::publish_zones(){

    update_tf();

    ZoneArray zones;
    zones.header.stamp = this->now();
    zones.header.frame_id = "base_link";

    for(int i = 0; i < fov_segments; i++){
        if(nearest_obstacles[i] == std::numeric_limits<double>::infinity()) continue;

        double a_l = i*angle_resolution - fov_angle/2; // left angle without padding
        double a_r = (i+1)*angle_resolution - fov_angle/2; // right angle without padding
        double r = nearest_obstacles[i];

        double a_padding = acos(zone_padding/r);
        a_l -= a_padding;
        a_r += a_padding;

        double r_near = r - zone_padding;
        double r_far = r + zone_padding;

        Zone z;
        z.max_speed = 0;

        Point32 nl, nr, fl, fr;
        nl.x = r_near*cos(a_l);
        nl.y = r_near*sin(a_l);
        nl.z = 0;
        nr.x = r_near*cos(a_r);
        nr.y = r_near*sin(a_r);
        nr.z = 0;
        fl.x = r_far*cos(a_l);
        fl.y = r_far*sin(a_l);
        fl.z = 0;
        fr.x = r_far*cos(a_r);
        fr.y = r_far*sin(a_r);
        fr.z = 0;

        z.poly.points.push_back(nl);
        z.poly.points.push_back(fl);
        z.poly.points.push_back(fr);
        z.poly.points.push_back(nr);

        transform_zone(z);

        zones.zones.push_back(z);
    }
    
    this->zone_pub->publish(zones);
}

void LidarObstacleDetectorNode::update_tf() {
    try {
		currentTf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
	} catch (tf2::TransformException& e) {
		RCLCPP_INFO(this->get_logger(), "Could not transform base_link to map: %s", e.what());
		return;
	}
}

void LidarObstacleDetectorNode::transform_zone(Zone& z){
    double tf_x = this->currentTf.transform.translation.x;
    double tf_y = this->currentTf.transform.translation.y;
    Eigen::Quaternion quat(this->currentTf.transform.rotation.w, this->currentTf.transform.rotation.x, this->currentTf.transform.rotation.y, this->currentTf.transform.rotation.z);
    auto rot_mat = quat.toRotationMatrix();

    for(size_t i = 0; i < z.poly.points.size(); i++){
        Eigen::Vector3d p(z.poly.points[i].x, z.poly.points[i].y, z.poly.points[i].z);
        p = rot_mat*p;

        z.poly.points[i].x =p[0] + tf_x;
        z.poly.points[i].y =p[1] + tf_y;
    }
}