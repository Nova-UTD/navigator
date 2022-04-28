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
    this->map_file_path = this->get_parameter("map_file_path").as_string();
    this->map = opendrive::load_map(this->map_file_path)->map;

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
    this->current_position_x = msg->pose.pose.position.x;
    this->current_position_y = msg->pose.pose.position.y;
    publish_odom();
}

void CurbLocalizerNode::publish_odom() {
    
    // find right curb centerline (left later)

    // curb detector looks ahead about 20m
    navigator::opendrive::LanePtr lane;
    if (current_orientation == 1) { //placeholder
        lane = navigator::opendrive::get_lane_from_xy(this->map, current_position_x + 20, current_position_y); 
    } else if (current_orientation == 2) {
        lane = navigator::opendrive::get_lane_from_xy(this->map, current_position_x - 20, current_position_y);
    } else if (current_orientation == 3) {
        lane = navigator::opendrive::get_lane_from_xy(this->map, current_position_x, current_position_y + 20);
    } else if (current_orientation == 4) {
        lane = navigator::opendrive::get_lane_from_xy(this->map, current_position_x, current_position_y - 20);
    }









    odom_out_pub->publish(*odom_out);
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

    Eigen::Vector3d translation = (icp.getFinalTransformation() * Eigen::Vector4d(0, 0, 0, 1)).block<3, 1>(0, 0);
    return translation;
}