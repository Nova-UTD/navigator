/*
 * Package:   curb_localizer
 * Filename:  CurbLocalizerNode.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include "curb_localizer/CurbLocalizerNode.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>

// weird build stuff
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

using namespace navigator::curb_localizer;

CurbLocalizerNode::CurbLocalizerNode() : Node("curb_localizer"){
    this->declare_parameter<std::string>("map_file_path", "data/maps/grand_loop/grand_loop.xodr");
    this->declare_parameter<double>("curb_look_distance", 20.0);


    this->map_file_path = this->get_parameter("map_file_path").as_string();
    this->map = opendrive::load_map(this->map_file_path)->map;

    this->get_parameter("curb_look_distance", this->look_distance);
    if(look_distance <= 0.0){
        RCLCPP_ERROR(this->get_logger(), "look_distance must be positive");
        look_distance = 20.0;
    }

    // Initialize the point clouds
    this->left_curb_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    this->right_curb_points = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>); 

    // curb detector class also outputs all candidate pts if necessary

    this->left_curb_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("curb_points/left",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CurbLocalizerNode::left_curb_points_callback, this, std::placeholders::_1));
    this->right_curb_points_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>("curb_points/right",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CurbLocalizerNode::right_curb_points_callback, this, std::placeholders::_1));
    this->odom_in_sub = this->create_subscription<nav_msgs::msg::Odometry>("/sensors/gnss/odom",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CurbLocalizerNode::odom_in_callback, this, std::placeholders::_1));
    this->odom_out_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_out",
        rclcpp::QoS(rclcpp::KeepLast(1)));
}

void convert_to_pcl(const sensor_msgs::msg::PointCloud2::SharedPtr msg, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::fromPCLPointCloud2(pcl_cloud, *out_cloud);
}

void CurbLocalizerNode::left_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    convert_to_pcl(msg, this->left_curb_points);
    flatten_cloud(this->left_curb_points, this->left_curb_points);
}

void CurbLocalizerNode::right_curb_points_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg){
    convert_to_pcl(msg, this->right_curb_points);
    flatten_cloud(this->right_curb_points, this->right_curb_points);
}

void CurbLocalizerNode::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    this->odom_in = msg;
    publish_odom();
}

void CurbLocalizerNode::publish_odom() {

    nav_msgs::msg::Odometry odom_out = *(this->odom_in);
    
    std::vector<std::vector<std::string>> path_roads = {
        {"81", "1"},
		{"953", "1"}
    };
    
    double current_position_x = this->odom_in->pose.pose.position.x;
    double current_position_y = this->odom_in->pose.pose.position.y;

    // get lane from current position
    std::shared_ptr<odr::Lane> current_lane = navigator::opendrive::get_lane_from_xy(map, current_position_x, current_position_y);

    // get road from current lane
    std::string road_id = current_lane->road.lock()->id;
    std::shared_ptr<odr::Road> current_road = map->roads[road_id];

    // get current s value
    double s = current_road->ref_line->match(current_position_x, current_position_y);    

    std::shared_ptr<odr::Road> target_road;
    std::shared_ptr<odr::LaneSection> target_lanesection;

    // if adding 20m to current s goes out of road bounds, then next road has curb

    if ((current_lane <= 0 && (s + 20) > current_road->length) || (current_lane > 0 && (s - 20) < 0.0)) {

        double next_s = abs(current_road->length - (s + 20));

        int i = 0;
        for (auto &stringVector : path_roads) {
            if (stringVector[0] == road_id && i != path_roads.size() - 1) {
                target_road = map->roads[path_roads[i + 1][0]];
                if (path_roads[i + 1][1] == "-1" || path_roads[i + 1][1] == "-2" || path_roads[i + 1][1] == "-3")  {
                    target_lanesection = target_road->get_lanesection(next_s);
                } else {
                    target_lanesection = target_road->get_lanesection(target_road->length - next_s);
                }
            }
            i++;
        }

    } else {
        target_road = current_road;
        target_lanesection = current_road->get_lanesection(s);
    }

    // iterate road lanes and find both curb lanes
    std::shared_ptr<odr::Lane> right_curb;
    std::shared_ptr<odr::Lane> left_curb;

    for (auto lane : target_lanesection->get_lanes()) {
        if (lane->type == "shoulder" && lane->id <= 0) {
            right_curb = lane;
        } else if (lane->type == "shoulder" && lane->id > 0) {
            left_curb = lane;
        }
    }

    // // get centerline for those lanes
    odr::Line3D right_curb_line = navigator::opendrive::get_centerline_as_xy(*right_curb, target_lanesection->s0, target_lanesection->get_end(), 0.25, false);
    odr::Line3D left_curb_line = navigator::opendrive::get_centerline_as_xy(*left_curb, target_lanesection->s0, target_lanesection->get_end(), 0.25, true);

    pcl::PointCloud<pcl::PointXYZ>::Ptr right_curb_points_map
        = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // TODO: curb point cloud in map frame from curb centerline

    Eigen::Vector3d displacement_right = find_translation(right_curb_points_map, right_curb_points);


    odom_out_pub->publish(odom_out);
}

/**
 * @brief Translates the given point cloud to the given pose.
 *  (car reference --> map reference)
 * @param in_cloud 
 * @param odom 
 * @param out_cloud 
 */
void CurbLocalizerNode::transform_points_to_odom(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    const nav_msgs::msg::Odometry &odom,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {

    Eigen::Affine3d odom_pose;
    odom_pose.rotate(Eigen::Quaterniond(odom.pose.pose.orientation.w,
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z));

    odom_pose.translate(Eigen::Vector3d(odom.pose.pose.position.x,
        odom.pose.pose.position.y,
        odom.pose.pose.position.z));

    pcl::transformPointCloud(*in_cloud, *out_cloud, odom_pose);
}

void CurbLocalizerNode::flatten_cloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud,
    pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
    // [ 1 0 0 ]   [ x ]   [ x ]
    // [ 0 1 0 ] * [ y ] = [ y ]
    // [ 0 0 0 ]   [ z ]   [ 0 ]
    Eigen::Affine3d projection_matrix = Eigen::Affine3d::Identity();
    projection_matrix(2, 2) = 0; // zero-indexed

    pcl::transformPointCloud(*in_cloud, *out_cloud, projection_matrix);
}

Eigen::Vector3d CurbLocalizerNode::find_translation(const pcl::PointCloud<pcl::PointXYZ>::Ptr truth,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr estimate) {
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(estimate);
    icp.setInputTarget(truth);
    icp.align(*estimate);

    Eigen::Vector3d translation = Eigen::Affine3d(icp.getFinalTransformation().cast<double>()).translation();
    return translation;
}