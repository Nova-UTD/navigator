#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <set>

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"
#include "Geometries/Line.h"
#include "RefLine.h"
#include "opendrive_utils/OpenDriveUtils.hpp"

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nova_msgs/msg/final_path.hpp>

class RouteItem {
public:
	RouteItem(std::string road_id, int lane_id) : road_id(road_id), lane_id(lane_id) {}
	std::string road_id;
	int lane_id;
};

class PathPublisherNode : public rclcpp::Node {
public:
	PathPublisherNode();
	void generatePaths();

private:
	std::set<std::string> all_ids;
	
	nova_msgs::msg::FinalPath path;
	navigator::opendrive::OpenDriveMapPtr map;
	double path_resolution; // The sampling interval taken along a lane's centerline to form the path
	visualization_msgs::msg::MarkerArray lane_markers;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	nav_msgs::msg::Odometry::SharedPtr cached_odom;
    rclcpp::Publisher<nova_msgs::msg::FinalPath>::SharedPtr paths_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    rclcpp::TimerBase::SharedPtr path_pub_timer{nullptr};

	nova_msgs::msg::FinalPath route1;
	nova_msgs::msg::FinalPath route2;

	void publish_paths_viz(nova_msgs::msg::FinalPath path);
	nova_msgs::msg::FinalPath generate_path(std::vector<std::string> &road_ids, std::vector<int> &lane_ids, navigator::opendrive::OpenDriveMapPtr map);
	void switch_path(nova_msgs::msg::FinalPath path);
};