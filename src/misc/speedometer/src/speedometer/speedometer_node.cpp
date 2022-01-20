/**
 * Package:     speedometer
 * Filename:    speedometer_node.cpp
 * Author:      Will Heitman
 * Email:       will.heitman@utdallas.edu
 * Copyright:   2022 Nova UTD
 * License:     MIT
 */

#include <cmath>

#include <iomanip>
#include <sstream>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;

class SpeedometerNode : public rclcpp::Node {

public:
    SpeedometerNode() : Node("speedometer_node") {
        this->declare_parameter<bool>("use_mph", false); // Use m/s by default

        odom_sub_ = this->create_subscription<Odometry>(
            "/odom", 10,
            [this](Odometry::SharedPtr odom_msg) {odomCb(odom_msg);}
        );

        marker_pub_ = this->create_publisher<Marker>("/speed_marker", 10);
    }

private:
    void odomCb(Odometry::SharedPtr odom_msg) {
        bool use_mph;
        this->get_parameter("use_mph", use_mph);

        auto lin_vel = odom_msg->twist.twist.linear;
        double speed = sqrt(
                            pow(lin_vel.x, 2) + 
                            pow(lin_vel.y, 2) +
                            pow(lin_vel.z, 2)
                        );

        RCLCPP_INFO(this->get_logger(), "Speed: %f", speed);

        Marker speed_marker;
        speed_marker.header.frame_id = "base_link";
        speed_marker.header.stamp = this->get_clock()->now();

        speed_marker.type = Marker::TEXT_VIEW_FACING;
        speed_marker.ns = "vehicle";
        speed_marker.id = 0;

        speed_marker.action = Marker::ADD;

        std::stringstream stream;

        if (use_mph == true)
            stream << std::fixed << std::setprecision(2) << speed * 2.237<< " mph";
        else
            stream << std::fixed << std::setprecision(2) << speed << " m/s";
        speed_marker.text = stream.str();

        Vector3 scale;
        scale.z = 1.0;
        speed_marker.scale = scale;

        ColorRGBA text_color;
        text_color.a = 1.0;
        text_color.r = 1.0;
        text_color.g = 1.0;
        text_color.b = 1.0;
        speed_marker.color = text_color;

        Point marker_pos;
        marker_pos.z = 2.0;
        speed_marker.pose.position = marker_pos;

        marker_pub_->publish(speed_marker);
    }

    rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<Marker>::SharedPtr marker_pub_;

};

int main(int argc, char * argv[]) {

    // Set everything up, including telling the ROS network
    // that a new node is starting up
    rclcpp::init(argc, argv);

    // Run our ROS node, including the sub and pub,
    // until told to stop
    rclcpp::spin(std::make_shared<SpeedometerNode>());

    // Tell the ROS network that our node is finished.
    // Releases pubs and subs.
    rclcpp::shutdown();

    return 0;
}