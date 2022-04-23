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
#include <voltron_msgs/msg/final_path.hpp>

class PathSection {
public:
    std::string road_id;
    int lane_id;
    double lanesection;
    double speed;
    PathSection(std::string road_id, int lane_id, double lanesection, double max_speed) : road_id(road_id), lane_id(lane_id), lanesection(lanesection), speed(max_speed) {}
    PathSection(std::string road_id, int lane_id, double lanesection) : road_id(road_id), lane_id(lane_id), lanesection(lanesection), speed(-1) {}
    PathSection(std::string road_id, int lane_id) : road_id(road_id), lane_id(lane_id), lanesection(0), speed(-1) {}
};

class PathPublisherNode : public rclcpp::Node {
public:
	PathPublisherNode();
	void generatePaths();

private:
	double path_resolution; // The sampling interval taken along a lane's centerline to form the path
	double cruising_speed; // The speed at which the vehicle drives 

	std::set<std::string> all_ids;
	
	voltron_msgs::msg::FinalPath path;
	navigator::opendrive::OpenDriveMapPtr map;

	visualization_msgs::msg::MarkerArray lane_markers;
    rclcpp::Publisher<voltron_msgs::msg::FinalPath>::SharedPtr paths_pub;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub;
    rclcpp::TimerBase::SharedPtr path_pub_timer{nullptr};

	voltron_msgs::msg::FinalPath route1;

	void publish_paths_viz(voltron_msgs::msg::FinalPath path);
	voltron_msgs::msg::FinalPath generate_path(std::vector<PathSection> &route_info, navigator::opendrive::OpenDriveMapPtr map);
	void switch_path(voltron_msgs::msg::FinalPath path);
};