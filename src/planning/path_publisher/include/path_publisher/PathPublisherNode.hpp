#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"
#include "Geometries/Line.h"
#include "RefLine.h"

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voltron_msgs/msg/costed_path.hpp>
#include <voltron_msgs/msg/costed_paths.hpp>

class PathPublisherNode : public rclcpp::Node {
public:
	PathPublisherNode();
	void generatePaths();

private:
	std::vector<std::string> onramp_sequence; // Road IDs
	std::vector<std::string> loop_sequence; // RoadIDs
	odr::OpenDriveMap* map;
	double path_resolution; // The sampling interval taken along a lane's centerline to form the path
	visualization_msgs::msg::MarkerArray lane_markers;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	nav_msgs::msg::Odometry::SharedPtr cached_odom;
    rclcpp::Publisher<voltron_msgs::msg::CostedPaths>::SharedPtr paths_pub;
    rclcpp::TimerBase::SharedPtr path_pub_timer{nullptr};
};