/*
 * Package:   curb_localizer
 * Filename:  CurbLocalizerNode.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include "curb_localizer/CurbLocalizerNode.hpp"
#include "opendrive_utils/OpenDriveUtils.hpp"


using namespace navigator::curb_localizer;

#define ROUTE_INFO_PARAMS "route_info_params"

CurbLocalizerNode::CurbLocalizerNode() : Node("curb_localizer"){
    this->declare_parameter<std::string>("map_file_path", "data/maps/grand_loop/grand_loop.xodr");
    this->declare_parameter<double>("max_bias_magnitude", 4);
    this->declare_parameter<std::vector<std::string>>(ROUTE_INFO_PARAMS, std::vector<std::string> ({}));
    
    this->get_parameter("max_bias_magnitude", MAX_BIAS_MAGNITUDE);
    this->map_file_path = this->get_parameter("map_file_path").as_string();
    this->map = opendrive::load_map(this->map_file_path)->map;

    std::vector<std::string> route_info_params;
	this->get_parameter<std::vector<std::string>>(ROUTE_INFO_PARAMS, route_info_params);

    route_info = std::vector<PathSection>();
	for (auto it = std::begin(route_info_params); it != std::end(route_info_params); it+=3)
		route_info.push_back(PathSection(*it, std::stoi(*(it+1)), std::stod(*(it+2))));

    // curb detector class also outputs all candidate pts if necessary
    this->odom_in_sub = this->create_subscription<nav_msgs::msg::Odometry>("/sensors/gnss/odom",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CurbLocalizerNode::odom_in_callback, this, std::placeholders::_1));
    this->curb_dist_sub = this->create_subscription<std_msgs::msg::Float32>("/curb_distance",
        rclcpp::QoS(rclcpp::KeepLast(1)),
        std::bind(&CurbLocalizerNode::curb_dist_callback, this, std::placeholders::_1));

    this->odom_out_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_out",
        rclcpp::QoS(rclcpp::KeepLast(1)));
}

void CurbLocalizerNode::odom_in_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
    this->odom_in = msg;
    publish_odom();
}

void CurbLocalizerNode::publish_odom() {

    if (odom_in == nullptr) return;
    if (map == nullptr) return;

    nav_msgs::msg::Odometry corrected_odom = *(this->odom_in);

    // Attempt to de-bias the odom using the accumulated known bias
    double& odo_x = corrected_odom.pose.pose.position.x;
    double& odo_y = corrected_odom.pose.pose.position.y;

    // find closest road to true odom data
    std::shared_ptr<odr::Road> current_road;
    int desired_lane_id;
    double min_dist = 99999;
    for (PathSection section : route_info) {
        std::shared_ptr<odr::Road> road = map->roads[section.road_id];
        double dist = navigator::opendrive::get_distance(road->ref_line, odo_x, odo_y);
        if (dist < min_dist) {
            min_dist = dist;
            current_road = road;
            desired_lane_id = section.lane_id;
        }
    }

    odo_x += this->bias_x;
    odo_y += this->bias_y;

    // If we don't have updated curb distance, don't do anything further
    if (this->dist_to_curb < 0) {
        this->odom_out_pub->publish(corrected_odom);
        return;
    }

    // get current s value
    double s = current_road->ref_line->match(odo_x, odo_y);
    std::shared_ptr<odr::LaneSection> target_lanesection = current_road->get_lanesection(s);

    if (target_lanesection == nullptr) {
        odom_out_pub->publish(corrected_odom);
        return;
    }

    navigator::opendrive::LanePtr curb_lane;
    for (auto lane : target_lanesection->get_lanes()) {
        if (lane->type == "shoulder" && (lane->id * desired_lane_id) >= 0)
            curb_lane = lane;
        // if we have a parking lane, supercedes actual shoulder 
        if (lane->type == "parking" && (lane->id * desired_lane_id) >= 0) {
            curb_lane = lane;
            break;
        }
    }

    if(curb_lane == nullptr) {
        // No curb found, cannot correct bias
        odom_out_pub->publish(corrected_odom);
        return; 
    }

    // Get the closest curb point, with for low curvature (as roads are) should
    // match the distance given to us by CurbDetector

    // Picked 0.1 because I wasn't sure what happens for identical 
    // start and end points, and picked 11 because 0.1 * 11 > 1.0
    // so at the very least there should be a point in the line even
    // if endpoints are not included. Not sure of any of this.
    auto curb = curb_lane->get_border_line(s, s+0.1, 11, false);
    double curb_x = curb[0][0];
    double curb_y = curb[0][1];

    // <dx, dy> represents the vector from car to curb
    double dx = curb_x - odo_x;
    double dy = curb_y - odo_y;
    double dist_to_curb_odom = sqrt(dx * dx + dy * dy);

    // We are looking at the right curb, so dy should be positive.
    // If not, it's a sign the odom has drifted to the wrong curb.

    // position + predicted_offset = curb pos = (position + bias) + measured_offset,
    // so bias = predicted_offset - measured_offset
    double bias_change_magnitude = dist_to_curb_odom - this->dist_to_curb;
    // All vectors are parallel, so the easiest way to find 
    // bias is bias = (bias_magnitude) * (normalized predicted_offset)
    double bias_coeff = bias_change_magnitude / dist_to_curb_odom;

    // <dx, dy> now represents the new bias vector (added on top of old bias)
    dx *= bias_coeff;
    dy *= bias_coeff;

    odo_y += dy;
    odo_x += dx;

    bias_x += dx;
    bias_y += dy;

    odom_out_pub->publish(corrected_odom);

    // Cleanup: ensure we don't reuse curb data,
    // and bound the bias magnitude
    this->dist_to_curb = -1;

    double bias_mag = sqrt(bias_x * bias_x + bias_y * bias_y);
    if (bias_mag > MAX_BIAS_MAGNITUDE) {
        bias_x *= MAX_BIAS_MAGNITUDE / bias_mag;
        bias_y *= MAX_BIAS_MAGNITUDE / bias_mag;
    }
}