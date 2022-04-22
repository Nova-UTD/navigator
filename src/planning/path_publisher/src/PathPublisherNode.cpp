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
	// this->declare_parameter<std::string>(MAP_PARAM, "data/maps/town07/Town07_Opt.xodr");
	this->declare_parameter<std::string>(MAP_PARAM, "data/maps/demo2/Demo2_map.xodr");
	this->declare_parameter<double>(SPEED_PARAM, 10.0);

	std::string xodr_path;
	this->get_parameter<std::string>(MAP_PARAM, xodr_path);
	this->get_parameter<double>(SPEED_PARAM, cruising_speed);

	paths_pub = this->create_publisher<FinalPath>("paths", 1);
	
	viz_pub = this->create_publisher<MarkerArray>("path_pub_viz", 1);
    
	/*auto route_1_road_ids = std::vector<std::string>{
		"21","39","57","584","7","693","6","509","5","4",
        "686","601","34","532","35","359","40","634","50","10","9","976",
        "36","210","46","436","59","168","60","464","61","559","62",
        "352","24","467","20","920",
	};
	auto route_1_lane_ids = std::vector<int> {
		-1,-1,-1,-1,1,1,1,1,1,1,
        1,-1,-1,-1,-1,-1,1,1,-3,-3,-3,-1,
        -1,-1,1,1,-1,-1,-1,-1,-1,-1,-1,
        -1,-1,-1,-1,-1,
	};*/
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
        PathSection("7", -4),
        PathSection("7", -4),
        PathSection("417", -1),
        PathSection("61", -1),
        PathSection("825", -1),
        PathSection("99", 4, 83.14),
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
		odr::Line3D centerline = navigator::opendrive::get_centerline_as_xy(*lane, lanesection->s0, lanesection->get_end(), step, lane_id>0);

		for (odr::Vec3D point : centerline) {
			route.push_back(point);
			costed_path.speeds.push_back(cruising_speed);
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
