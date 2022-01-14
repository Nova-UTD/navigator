/*
 * Package:   nova_gps
 * Filename:  GPSInterfaceNode.cpp
 * Author:    Avery Bainbridge
 * Email:     axb200157@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include "nova_gps/GPSInterface.hpp"
#include "nova_gps/ConcreteGPSInterface.hpp"
#include "nova_gps/GPSInterfaceNode.hpp"

using namespace std::chrono_literals;
using Nova::GPS::GPSInterfaceNode;
using std::placeholders::_1;

const auto receive_frequency = 50ms;

GPSInterfaceNode::GPSInterfaceNode(const std::string & interface_name) // The interface name should also definitely be a ROS param, like above. WSH.
  : Node("gps_interface_node") {

  this->gps_interface = std::make_unique<Nova::GPS::ConcreteGPSInterface>(interface_name);
  this->pose_timer = this->create_wall_timer
    (receive_frequency, bind(& GPSInterfaceNode::send_pose, this));
  this->odometry_publisher = this->create_publisher<nav_msgs::msg::Odometry>("/gps/odometry", 10);
}

GPSInterfaceNode::~GPSInterfaceNode() {
    
}
// convert a (d)ddmm.mmmmm to decimal degrees
double nmea_to_deg(std::string & nmea) {
  double value = NAN;
  // validation ( we might get empty strings if there is no lock )
  if(nmea.length() > 5) {
    //   d d d m|m|. m m m m m
    //   d d m m|.|m m m m m
    //           ^
    // a decimal point (or lack thereof) in this column tells us how many digits the degree part has
    int digits = nmea[4] == '.' ? 2 : 3;
    // (d) d d
    double degrees = std::stod(nmea.substr(0, digits));
    // m m . m m m m m, and there are 60 minutes to a degree
    double minutes = std::stod(nmea.substr(digits, nmea.length() - digits)) / 60.0;
    value = degrees + minutes;
  }
  return value;
}
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
Nova::GPS::HNRPVT parse_hnrpvt(const std::unique_ptr<Nova::GPS::UBXMessage> msg) {
  Nova::GPS::HNRPVT st = {
    .iTOW = msg->data->read_dword(),
    .year = msg->data->read_word(),
    .month = msg->data->read_byte(),
    .day = msg->data->read_byte(),
    .hour = msg->data->read_byte(),
    .min = msg->data->read_byte(),
    .sec = msg->data->read_byte(),
    .valid = msg->data->read_byte(),
    .nano = msg->data->read_signed_int(),
    .gpsFix = msg->data->read_byte(),
    .flags = msg->data->read_byte(),
    ._1 = msg->data->read_byte(),
    ._2 = msg->data->read_byte(),
    .lon = msg->data->read_signed_int(),
    .lat = msg->data->read_signed_int(),
    .height = msg->data->read_signed_int(),
    .hMSL = msg->data->read_signed_int(),
    .gSpeed = msg->data->read_signed_int(),
    .speed = msg->data->read_signed_int(),
    .headMot = msg->data->read_signed_int(),
    .headVeh = msg->data->read_signed_int(),
    .hAcc = msg->data->read_dword(),
    .vAcc = msg->data->read_dword(),
    .sAcc = msg->data->read_dword(),
    .headAcc = msg->data->read_dword(),
    .__1 = msg->data->read_byte(),
    .__2 = msg->data->read_byte(),
    .__3 = msg->data->read_byte(),
    .__4 = msg->data->read_byte(),
  };
  return st;
}
#pragma GCC diagnostic pop
void GPSInterfaceNode::send_pose() {
  bool found = false;
  std::unique_ptr<UBXMessage> raw_msg;

  this->gps_interface->gather_messages();

  while(this->gps_interface->has_message()) {
      auto msg = this->gps_interface->get_message();
      if(msg->id == 0x00 && msg->mclass == 0x28) {
        raw_msg = std::move(msg);
      }
  }
  if(found) {
    // message.header.frame_id = "/earth";
    // for correctness, we should use the utc from the message.

    HNRPVT hnrpvt = parse_hnrpvt(std::move(raw_msg));
    
    double lat = hnrpvt.lat / (double)1e7;
    double lon = hnrpvt.lon / (double)1e7;

    double altitude = hnrpvt.hMSL / 1000.0;

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // math formula time
    // WGS-84 major axis
    const double a = 6378137.0;
    // WGS-84 minor axis
    const double b = 6356752.314245;
    // WGS-84 1/flattening
    // const double f_inv = 298.257223563;

    const double e_sq = (a * a - b * b) / (a * a);

    double N = a / sqrt(1 - e_sq * sin(lat_rad) * sin(lat_rad));

    double cos_lat = cos(lat_rad);
    double sin_lat = sin(lat_rad);

    double x = (N + altitude) * cos_lat * cos(lon_rad);
    double y = (N + altitude) * cos_lat * sin(lon_rad);
    double z = ((1 - e_sq)*N + altitude) * sin_lat;
    geometry_msgs::msg::Point gps_position;
    gps_position.x = x;
    gps_position.y = y;
    gps_position.z = z;
    // message.pose.pose.position = point;
    // utterly useless, but let's control what we send
    message.pose.covariance = {1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1};

    double heading_deg = hnrpvt.headVeh / (double)1e5;
    double heading_rad = heading_deg * M_PI / 180.0;

    // this->publisher->publish(message);

    // std_msgs::msg::Float64 hdg;
    // std_msgs::msg::Float64 vel;

    // hdg.data = heading_rad;

    double speed = hnrpvt.gSpeed / 1000.0;

    // this->heading_publisher->publish(hdg);
    // this->velocity_publisher->publish(vel);
    
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.frame_id = "/earth";
    odom_msg.header.stamp = rclcpp::Clock().now();

    odom_msg.pose.pose.position = gps_position;

    odom_msg.pose.pose.orientation.x = 0;
    odom_msg.pose.pose.orientation.y = 0;
    odom_msg.pose.pose.orientation.z = sin(heading_rad); // This should do the trick. 🤞 WSH.
    odom_msg.pose.pose.orientation.w = cos(heading_rad); // <w,x,y,z> = cos(θ)+sin(θ)<i,j,k>

    // TODO: Add angular components from GPS IMU. WSH.
    odom_msg.twist.twist.linear.x = sin(heading_rad) * speed;
    odom_msg.twist.twist.linear.y = cos(heading_rad) * speed;
    odom_msg.twist.twist.linear.z = 0; // Assume that the car is traveling on a level plane.
    RCLCPP_INFO(get_logger(), "Publishing GPS message.");
    this->odometry_publisher->publish(odom_msg);
  }
}
