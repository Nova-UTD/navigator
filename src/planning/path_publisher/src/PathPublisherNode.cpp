#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

#include <iostream>
#include <fstream>
#include <iomanip>

#include <ctime>
#include <cstdlib> 

using namespace std;

#include "path_publisher/PathPublisherNode.hpp"
#include "opendrive_utils/OpenDriveUtils.hpp"

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using voltron_msgs::msg::FinalPath;
using namespace std::chrono_literals;

#define MAP_PARAM "map_xodr_file_path"
#define SPEED_PARAM "cruising_speed_ms"

PathPublisherNode::PathPublisherNode() : Node("path_publisher_node") {

	// Declare parameters
	this->declare_parameter<std::string>(MAP_PARAM, "data/maps/demo2/Demo2_map.xodr");
	this->declare_parameter<double>(SPEED_PARAM, 10.0);

	std::string xodr_path;
	this->get_parameter<std::string>(MAP_PARAM, xodr_path);
	this->get_parameter<double>(SPEED_PARAM, cruising_speed);

	paths_pub = this->create_publisher<FinalPath>("paths", 1);
	
	viz_pub = this->create_publisher<MarkerArray>("path_pub_viz", 1);
    
    auto route_info = std::vector<PathSection> {
        PathSection("42", -1),
        PathSection("21", 1),
        PathSection("80", 1),
        PathSection("540", 1),
        PathSection("79", 1),
        PathSection("756", 1),
        PathSection("119", -1, 0),
        PathSection("119", -1, 70.91),
        PathSection("967", -1),
        PathSection("73", 1, 35.33),
        PathSection("73", 2, 0),
        PathSection("342", 1),
        PathSection("104", -1),
        PathSection("945", -1),
        PathSection("105", -1),
        PathSection("875", -1),
        PathSection("39", -1),
        PathSection("739", -1),
        PathSection("76", -1, 0),
        PathSection("76", -1, 0.39),
        PathSection("76", -1, 55.67),
        PathSection("403", -1),
        PathSection("7", -4),
        PathSection("7", -4, 31.55),
        PathSection("7", -4, 36.77),
        PathSection("417", -1),
        PathSection("61", -1),
        PathSection("825", -1),
        PathSection("99", 4, 83.14),
        PathSection("99", 5, 60.17),
        PathSection("99", 5, 53.57),
        PathSection("99", 4, 3.75),
        PathSection("99", 4, 0),
        PathSection("562", 1),
        PathSection("43", -1, 0),
        PathSection("43", -1, 53.94),
        PathSection("919", -1),
        PathSection("47", -1, 0),
        PathSection("47", -1, 24.30),
        PathSection("47", -1, 33.00),
        PathSection("47", -2, 53.36),
        PathSection("47", -2, 64.47),
        PathSection("261", -1),
        PathSection("86", -1, 0),
        PathSection("86", -1, 37.88),
        PathSection("110", -1),
        PathSection("679", -1),
        PathSection("111", -1),
        PathSection("534", -1, 0),
        PathSection("534", -2, 4.09),
        PathSection("112", -2),
        PathSection("688", -1),
        PathSection("55", 1),
        PathSection("180", 1), //maybe 186
        PathSection("54", 1),
        PathSection("982", 1),
        PathSection("17", -1),
        PathSection("254", -1),
        PathSection("18", -1),
        PathSection("1002", -1),
        PathSection("19", -1),
        PathSection("924", -1),
        PathSection("20", -1),
        PathSection("839", -1),
        PathSection("83", -3),
        PathSection("28", -3),
        PathSection("0", -3),
        PathSection("130", -1),
    };

    for (auto s : route_info) {
        all_ids.insert(s.road_id);
    }
	path_pub_timer = this->create_wall_timer(0.5s, std::bind(&PathPublisherNode::generatePaths, this));

	// Read map from file, using our path param
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	map = navigator::opendrive::load_map(xodr_path)->map;

	

	this->route1 = generate_path(route_info, map);
	this->path = this->route1;
}

voltron_msgs::msg::FinalPath PathPublisherNode::generate_path(std::vector<PathSection> &route_info, navigator::opendrive::OpenDriveMapPtr map)
{
	std::vector<odr::Vec3D> route;
	FinalPath costed_path;
	double step = 0.25;
	for (auto& section : route_info) {
		std::string id = section.road_id;
		int lane_id = section.lane_id;
		auto road = map->roads[id];
		//there is only one lanesection per road on this map
		std::shared_ptr<odr::LaneSection> lanesection = road->get_lanesection(section.lanesection);
		odr::LaneSet laneset = lanesection->get_lanes();
		std::shared_ptr<odr::Lane> lane = nullptr;
        //loop through the laneset to find a pointer to the lane.
		for (auto l : laneset) {
			if (l->id == lane_id) {
				lane = l;
				break;
			}
		}
		if (lane == nullptr) {
			RCLCPP_WARN(this->get_logger(), "NO LANE FOR ROAD %s", id.c_str());
			continue;
		}
        if (lanesection == nullptr) {
			RCLCPP_WARN(this->get_logger(), "NO LANESECTION FOR ROAD %s", id.c_str());
			continue;
		}
        RCLCPP_WARN(this->get_logger(), "NO LANE FOR ROAD %s", id.c_str());
		odr::Line3D centerline = navigator::opendrive::get_centerline_as_xy(*lane, lanesection->s0, lanesection->get_end(), step, lane_id>0);

		for (odr::Vec3D point : centerline) {
			route.push_back(point);
            //if a speed is defined, use that over the default cruising speed
			costed_path.speeds.push_back(section.speed < 0 ? cruising_speed : section.speed);
		}
	}
	RCLCPP_INFO(this->get_logger(), "generated path");

	
	size_t count = 0;
	for (odr::Vec3D pt3d : route) {
		Point path_pt;
		path_pt.x = pt3d[0];
		path_pt.y = pt3d[1];
		path_pt.z = pt3d[2];
		
		count ++;
		costed_path.points.push_back(path_pt);
	}
	return costed_path;
}

//shamelessly stolen from egan
void PathPublisherNode::publish_paths_viz(FinalPath path)
{
	MarkerArray marker_array;
	Marker marker;

	// Set header and identifiers
	marker.header.frame_id = "map";
	// marker.header.stamp = this->now();
	marker.ns = "path_pub_viz";
    marker.frame_locked = true;
	marker.id = 1;

	// Add data contents
	marker.type = Marker::LINE_STRIP;
	marker.action = Marker::ADD;
	marker.points = path.points;

	// Set visual display. Not sure if this is needed
	marker.scale.x = 1;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 1.0;
	marker.color.b = 0;

	// Add path to array
	marker_array.markers.push_back(marker);
	// RCLCPP_INFO(this->get_logger(), "path viz");

	viz_pub->publish(marker_array);
}


void PathPublisherNode::generatePaths() {

	paths_pub->publish(this->path);
	publish_paths_viz(this->path);
	
}