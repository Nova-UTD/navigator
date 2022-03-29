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

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voltron_msgs/msg/final_path.hpp>

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
	std::set<std::string> onramp_ids; // Road IDs
	std::set<std::string> loop_ids; // RoadIDs
	std::set<std::string> all_ids;
	std::set<std::string> stop_road_ids;
	std::set<std::string> go_road_ids;
	
	voltron_msgs::msg::FinalPath path;
	odr::OpenDriveMap* map;
	double path_resolution; // The sampling interval taken along a lane's centerline to form the path
	visualization_msgs::msg::MarkerArray lane_markers;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	nav_msgs::msg::Odometry::SharedPtr cached_odom;
    rclcpp::Publisher<voltron_msgs::msg::FinalPath>::SharedPtr paths_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    rclcpp::TimerBase::SharedPtr path_pub_timer{nullptr};

	voltron_msgs::msg::FinalPath route1;
	voltron_msgs::msg::FinalPath route2;
	voltron_msgs::msg::FinalPath route2_nonstop;

	void publish_paths_viz(voltron_msgs::msg::FinalPath path);
	voltron_msgs::msg::FinalPath generate_path(std::vector<std::string> &road_ids, std::vector<int> &lane_ids, odr::OpenDriveMap *map, std::set<std::string> &stop_roads);
	void switch_path(voltron_msgs::msg::FinalPath path);
};