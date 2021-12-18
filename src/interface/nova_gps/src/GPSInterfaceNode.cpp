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

const auto receive_frequency = 1s;

GPSInterfaceNode::GPSInterfaceNode(const std::string & interface_name)
  : Node("gps_interface_node") {

  this->gps_interface = std::make_unique<Nova::GPS::ConcreteGPSInterface>(interface_name);
  this->pose_timer = this->create_wall_timer
    (receive_frequency, bind(& GPSInterfaceNode::send_pose, this));
  this->publisher = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>
    ("gps/pose", 64);
  this->heading_publisher = this->create_publisher<std_msgs::msg::Float64>
    ("gps/heading", 64);
  this->velocity_publisher = this->create_publisher<std_msgs::msg::Float64>
    ("gps/velocity", 64);
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

// todo: remove when NMEAMessage is split upstream
inline void tokenize(Nova::GPS::NMEAMessage msg, std::vector<std::string> & vec) {
  int sub_start = 0;
  int sub_end = msg.find(',');
  while(sub_end != -1) {
    vec.push_back(msg.substr(sub_start + 1, sub_end - sub_start - 1));
    sub_start = sub_end;
    sub_end = msg.find(',', sub_start + 1);
  }
}

void GPSInterfaceNode::send_pose() {
  bool found_gga = false;
  bool found_vtg = false;
  NMEAMessage gga;
  NMEAMessage vtg;

  this->gps_interface->gather_messages();

  while(this->gps_interface->has_nmea_message()) {
      NMEAMessage msg = *this->gps_interface->get_nmea_message();
      if(!found_gga && msg.substr(3, 3) == "GGA") {
        gga = msg;
        found_gga = true;
      }
      if(!found_vtg && msg.substr(3, 3) == "VTG") {
        vtg = msg;
        found_vtg = true;
      }
  }
  if(found_gga && found_vtg) {
    geometry_msgs::msg::PoseWithCovarianceStamped message;
    message.header.frame_id = "/earth";
    // for correctness, we should use the utc from the message.
    message.header.stamp = rclcpp::Clock().now();
    std::vector<std::string> frames_gga;
    std::vector<std::string> frames_vtg;

    tokenize(gga, frames_gga);
    tokenize(vtg, frames_vtg);

    // todo: replace all these magic frame numbers with proper accesses, e.g.
    // double lat = gga->latitude;
    
    if(frames_gga[6] == "0") {
      // reject as no lock.
      return;
    }
    // warning: we assume NW
    double lat = nmea_to_deg(frames_gga[2]);
    double lon = -nmea_to_deg(frames_gga[4]);

    double altitude = std::stod(frames_gga[9]);

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
    geometry_msgs::msg::Point point;
    point.x = x;
    point.y = y;
    point.z = z;
    message.pose.pose.position = point;
    // utterly useless, but let's control what we send
    message.pose.covariance = {1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1};

    double heading_deg = frames_vtg[1] == "" ? NAN : std::stod(frames_vtg[1]);
    double heading_rad = heading_deg * M_PI / 180.0;
    
    // todo: proper quaternion rotation via axis-angle?

    message.pose.pose.orientation.x = 0;
    message.pose.pose.orientation.y = 0;
    message.pose.pose.orientation.z = 0;
    message.pose.pose.orientation.w = 1;

    this->publisher->publish(message);

    std_msgs::msg::Float64 hdg;
    std_msgs::msg::Float64 vel;

    hdg.data = heading_rad;

    const double KNOTS_TO_MPS = 0.5144456333854638;
    vel.data = std::stod(frames_vtg[5]) * KNOTS_TO_MPS;

    this->heading_publisher->publish(hdg);
    this->velocity_publisher->publish(vel);
    
  }
}
