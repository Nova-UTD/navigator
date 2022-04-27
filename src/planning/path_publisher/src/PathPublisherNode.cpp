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
#define SPEED_BUMP_PARAM "speed_bump_ms"

PathPublisherNode::PathPublisherNode() : Node("path_publisher_node") {

	// Declare parameters
	this->declare_parameter<std::string>(MAP_PARAM, "data/maps/grand_loop/grand_loop.xodr");
	this->declare_parameter<double>(SPEED_PARAM, 10.0);
    this->declare_parameter<double>(SPEED_BUMP_PARAM, 2.0);

	std::string xodr_path;
	this->get_parameter<std::string>(MAP_PARAM, xodr_path);
	this->get_parameter<double>(SPEED_PARAM, cruising_speed);
    this->get_parameter<double>(SPEED_BUMP_PARAM, speed_bump_speed);

	paths_pub = this->create_publisher<FinalPath>("paths", 1);
	
	viz_pub = this->create_publisher<MarkerArray>("path_pub_viz", 1);
    
    auto route_info = std::vector<PathSection> {
        PathSection("82", -1),
        PathSection("919", -1),
        PathSection("105", -1),
        PathSection("332", -1),
        PathSection("39", -1),
        PathSection("663", -1),
        PathSection("78", -1),
        PathSection("663", -1),
        PathSection("337", -1),
        PathSection("7", -1, 0),
        PathSection("7", -1, 27.96),
        PathSection("7", -1, 33.18),
        PathSection("7", -1),
        PathSection("465", -1),
        PathSection("63", -1),
        PathSection("776", -1),
        PathSection("98", 4, 82.99),
        PathSection("98", 5, 60.03),
        PathSection("98", 6, 53.43),
        PathSection("98", 5, 3.61),
        PathSection("98", 5, 0),
        PathSection("582", 1),
        PathSection("98", 5, 0),
        PathSection("43", -1, 0),
        PathSection("43", -1, 59.72),
        PathSection("898", -1),
        PathSection("48", -1, 0),
        PathSection("48", -1, 20.06),
        PathSection("48", -1, 28.76),
        PathSection("48", -2, 49.12),
        PathSection("48", -2, 60.23),
        PathSection("262", -1),
        PathSection("86", -1),
        PathSection("998", -1),
        PathSection("110", -1),
        PathSection("689", -1),
        PathSection("111", -1, 0),
        PathSection("111", -2, 106.88),
        PathSection("219", -2),
        PathSection("113", -2),
        PathSection("609", -1),
        PathSection("57", 1),
        PathSection("390", 1),
        PathSection("55", 1, 263.97),
        PathSection("55", 1, 0),


    };

    for (auto s : route_info) {
        all_ids.insert(s.road_id);
    }
	path_pub_timer = this->create_wall_timer(10s, std::bind(&PathPublisherNode::generatePaths, this));

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