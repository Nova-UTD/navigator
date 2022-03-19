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

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using voltron_msgs::msg::CostedPath;
using voltron_msgs::msg::CostedPaths;
using namespace std::chrono_literals;

PathPublisherNode::PathPublisherNode() : Node("path_publisher_node") {

	this->declare_parameter<std::string>("xodr_path", "/home/main/navigator/data/maps/town10/Town10HD_Opt.xodr");
	this->declare_parameter<double>("path_resolution", 2.0);
	paths_pub = this->create_publisher<CostedPaths>("paths", 1);
	odom_sub = this->create_subscription<Odometry>("/odometry/filtered", 1, [this](Odometry::SharedPtr msg) {
		cached_odom = msg;
	});

	// int roads[] = {20,875,21,630,3,0,10,17,7,90,6,735,5,516,4,8,1,765,2,566,3,0};
	onramp_ids = std::set<std::string>{
		"20","875","21","630", "3"
	};
	loop_ids = std::set<std::string>{
		"3","0","10","17","7","90","6","735",
		"5","516","4","8","1","675","2","630"
	};
	all_ids = std::set<std::string>{
		"20","875","21","630","3","0","10","17","7","90","6","735",
		"5","516","4","8","1","675","2","630"
	};

	path_pub_timer = this->create_wall_timer(0.5s, std::bind(&PathPublisherNode::generatePaths, this));

	// Read map from file, using our path param
	std::string xodr_path = this->get_parameter("xodr_path").as_string();
	path_resolution = this->get_parameter("path_resolution").as_double();
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	map = new odr::OpenDriveMap(xodr_path, true, true, false, true);

	// // Let's build our two RoadSets
	// auto all_roads = map->get_roads();
	// for (auto road : all_roads) {
	// 	if(onramp_ids.find(road->id) != onramp_ids.end()) {
	// 		onramp_sequence.insert(road);
	// 	}
	// 	if(loop_ids.find(road->id) != loop_ids.end()) {
	// 		loop_sequence.insert(road);
	// 	}
	// 	if(all_ids.find(road->id) != all_ids.end()) {
	// 		all_roads_on_route.insert(road);
	// 	}
	// }
	// RCLCPP_INFO(this->get_logger(), "Onramp has %i roads, loop has %i roads ", onramp_sequence.size(), loop_sequence.size());
}

/**
 * PSEUDOCODE
 * 1. Find current lane + road ID
 * 2. If within road on onramp (moving onto loop, so road ID is not within "loop sequence"):
 * 	a. Find "s" of car on current road
 * 	b. Sample lane centerline at 1 meter intervals within current road, from "s" to end. Append points to path
 * 	c. For each remaining road in onramp seq, sample lane centerline and append points to path
 * 
 */

void PathPublisherNode::generatePaths() {

	// Wait until odometry data is available
	if (cached_odom == nullptr) {
		RCLCPP_WARN(get_logger(), "Odometry not yet received, skipping...");
		return;
	} else {
		RCLCPP_INFO(get_logger(), "Generating paths...");
	}

	CostedPaths costed_paths;
	CostedPath costed_path;
	costed_paths.header.frame_id = 'map';
	costed_paths.header.stamp = get_clock()->now();

	Point current_pos = cached_odom->pose.pose.position;

	RCLCPP_INFO(get_logger(), "Finding closest lane.");

	auto currentLane = map->get_lane_from_xy_with_route(current_pos.x, current_pos.y, all_ids);
	if (currentLane == nullptr)
		return;
	// auto currentLane = map->get_lane_from_xy(current_pos.x, current_pos.y);
	auto currentRoad = currentLane->road.lock();
	auto refline = currentRoad->ref_line;
	double s = refline->match(current_pos.x, current_pos.y);

	if (currentLane == nullptr) {
		RCLCPP_WARN(get_logger(), "Lane could not be located.");
		return;
	}
	RCLCPP_INFO(get_logger(), "Current lane: %i", currentLane->id);
	odr::Line3D centerline;
	// RCLCPP_INFO(get_logger(), "Getting centerline.");
	if (currentLane->id < 0) {
		RCLCPP_INFO(get_logger(), "Getting neg. centerline.");
		centerline = currentLane->get_centerline_as_xy(s+1.0, refline->length, 1.0);
	} else {
		RCLCPP_INFO(get_logger(), "Getting pos. centerline.");
		centerline = currentLane->get_centerline_as_xy(currentLane->lane_section.lock()->s0, s-1.0, 1.0);
		// centerline = currentLane->get_centerline_as_xy(refline->length, s-1.0, 1.0);
	}
	RCLCPP_INFO(get_logger(), "Got centerline.");
	for (odr::Vec3D pt3d : centerline) {
		// RCLCPP_INFO(get_logger(), "%f, %f", pt3d[0], pt3d[1]);
		Point path_pt;
		path_pt.x = pt3d[0];
		path_pt.y = pt3d[1];
		path_pt.z = pt3d[2];
		
		costed_path.points.push_back(path_pt);
	}


	// if (costed_path.points.size() < 10) { // If path is short, add successor lane's points
	// 	double end_dx = (costed_path.points.at(costed_path.points.size()-1).x - costed_path.points.at(costed_path.points.size()-2).x);
	// 	double end_dy = (costed_path.points.at(costed_path.points.size()-1).y - costed_path.points.at(costed_path.points.size()-2).y);
	// 	double next_x = costed_path.points.at(costed_path.points.size()-1).x + end_dx;
	// 	double next_y = costed_path.points.at(costed_path.points.size()-1).y + end_dy;
	// 	currentLane = currentLane = map->get_lane_from_xy_with_route(next_x, next_y, all_ids);
	// 	if (currentLane == nullptr)
	// 		return;
	// 	if (currentLane->id < 0) {
	// 		RCLCPP_INFO(get_logger(), "Getting neg. centerline.");
	// 		centerline = currentLane->get_centerline_as_xy(s+1.0, refline->length, 1.0);
	// 	} else {
	// 		RCLCPP_INFO(get_logger(), "Getting pos. centerline.");
	// 		centerline = currentLane->get_centerline_as_xy(s+1.0, refline->length, 1.0);
	// 		// centerline = currentLane->get_centerline_as_xy(refline->length, s-1.0, 1.0);
	// 	}
	// 	for (odr::Vec3D pt3d : centerline) {
	// 		// RCLCPP_INFO(get_logger(), "%f, %f", pt3d[0], pt3d[1]);
	// 		Point path_pt;
	// 		path_pt.x = pt3d[0];
	// 		path_pt.y = pt3d[1];
	// 		path_pt.z = pt3d[2];
			
	// 		costed_path.points.push_back(path_pt);
	// 	}
	// }

	costed_paths.paths.push_back(costed_path);
	

	paths_pub->publish(costed_paths);
}