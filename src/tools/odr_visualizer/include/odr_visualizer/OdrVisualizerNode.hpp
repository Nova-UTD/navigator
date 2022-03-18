#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"
#include "RefLine.h"

// Geometry stuff
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// struct LaneBoostPoly {
// 	std::string id;
// 	typedef boost::geometry::model::polygon<point_type> poly;
// };

class OdrVisualizerNode : public rclcpp::Node {
public:
	OdrVisualizerNode();
	void publishMarkerArray();
	void checkCurrentLane();
	// bool isWithin()

private:
	visualization_msgs::msg::MarkerArray lane_markers;
	int point_count;
	odr::OpenDriveMap odr_map = odr::OpenDriveMap("", true, true, false, true);
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	nav_msgs::msg::Odometry cached_odom_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr map_pub_timer{nullptr};
	rclcpp::TimerBase::SharedPtr current_lane_timer{nullptr};
};