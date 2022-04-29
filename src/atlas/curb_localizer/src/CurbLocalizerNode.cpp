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

#define print(M) RCLCPP_INFO(this->get_logger(), M)

CurbLocalizerNode::CurbLocalizerNode() : Node("curb_localizer"){
    this->declare_parameter<std::string>("map_file_path", "data/maps/grand_loop/grand_loop.xodr");
    
    this->map_file_path = this->get_parameter("map_file_path").as_string();
    this->map = opendrive::load_map(this->map_file_path)->map;

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
    odom_in->pose.pose.position.x += bias_x;
    odom_in->pose.pose.position.y += bias_y;
    print("Odom cb");
    publish_odom();
}

void CurbLocalizerNode::publish_odom() {

    if (odom_in == nullptr) return;
    if (map == nullptr) return;

    nav_msgs::msg::Odometry odom_out = *(this->odom_in);

    double current_position_x = this->odom_in->pose.pose.position.x;
    double current_position_y = this->odom_in->pose.pose.position.y;

    // get lane from current position
    std::shared_ptr<odr::Lane> current_lane = navigator::opendrive::get_lane_from_xy(map, current_position_x, current_position_y);

    if (current_lane == nullptr){
        odom_out_pub->publish(odom_out);
        return;
    } 

    // get road from current lane
    std::string road_id = current_lane->road.lock()->id;
    std::shared_ptr<odr::Road> current_road = map->roads[road_id];

    if (current_road == nullptr){
        odom_out_pub->publish(odom_out);
        return;
    } 

    // get current s value
    double s = current_road->ref_line->match(current_position_x, current_position_y);

    std::shared_ptr<odr::Road> target_road;
    std::shared_ptr<odr::LaneSection> target_lanesection;

    // if adding 20m to current s goes out of road bounds, then next road has curb
    target_road = current_road;
    target_lanesection = current_road->get_lanesection(s);

    if (target_lanesection == nullptr){
        odom_out_pub->publish(odom_out);
        return;
    }

    double current_lane_id = current_lane->id;
    navigator::opendrive::LanePtr curb_lane;
    for (auto lane : target_lanesection->get_lanes()) {
        if (lane->type == "shoulder" && (lane->id * current_lane->id) >= 0 )
            curb_lane = lane;
    }
    if(curb_lane == nullptr) return; // No curb found

    // Get t of curb at current 


    // get centerline for those lanes
    odr::Line3D curb_line = navigator::opendrive::get_centerline_as_xy(*curb_lane, target_lanesection->s0, target_lanesection->get_end(), 0.25, false);

    auto curb_line_it = curb_line.begin();
    double dist_traversed = 0;
    auto last_pt = *curb_line_it;
    for (; curb_line_it != curb_line.end(); curb_line_it++) {
        auto curr_pt = *curb_line_it;
        double dist = 0;
        for(int  i : {0,1,2}){
            double dxi = curr_pt[i] - last_pt[i];
            dist += dxi * dxi;
        }
        dist = sqrt(dist);
        if(dist + dist_traversed > s){
            // Interpolate point 
            double interp_factor = (s - dist_traversed) / dist;
            for(int i : {0,1,2}){
                double dxi = (curr_pt[i] - last_pt[i]) * interp_factor;
                last_pt[i] += dxi;
            }
            break;
        } 
        dist_traversed += dist;
        last_pt = curr_pt;
    }

    // last_pt should contain the point at coordinate s
    // TODO replace this with message topic
    double dx = last_pt[0] - current_position_x;
    double dy = last_pt[1] - current_position_y;
    double dist_to_curb_odom = sqrt(dx * dx + dy * dy);

    double interp_coeff = this->dist_to_curb / dist_to_curb_odom;

    dx *= interp_coeff;
    dy *= interp_coeff;

    odom_out.pose.pose.position.x += dx;
    odom_out.pose.pose.position.y += dy;

    bias_x += dx;
    bias_y += dy;

    odom_out_pub->publish(odom_out);
}