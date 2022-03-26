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
using voltron_msgs::msg::FinalPath;
using namespace std::chrono_literals;

PathPublisherNode::PathPublisherNode() : Node("path_publisher_node") {

	this->declare_parameter<std::string>("xodr_path", "/home/main/navigator/data/maps/town10/Town10HD_Opt.xodr");
	this->declare_parameter<double>("path_resolution", 2.0);
	paths_pub = this->create_publisher<FinalPath>("paths", 1);
	odom_sub = this->create_subscription<Odometry>("/odometry/filtered", 1, [this](Odometry::SharedPtr msg) {
		cached_odom = msg;
	});
	viz_pub = this->create_publisher<MarkerArray>("path_pub_viz", 1);

	auto route_1_road_ids = std::vector<std::string>{
		"20","875","21","630","3",
		"0","10","17","7","90",
		"6","735","5","516","4",
		"8","1"//,"675","2","566"
	};
	auto route_1_lane_ids = std::vector<int> {
		-1, -1, -1, -1, -2,
		-2, -2, -2, 2, 2,
		2, 2, 2, 2, 2,
		-2, -2, //-2, -2, -2
	};
	this->go_road_ids = std::set<std::string> {
		"6", "5", "675", "2", "3"
	};
	this->stop_road_ids = std::set<std::string> {
		"90", "735", "566"
	};
	auto route_2_road_ids = std::vector<std::string>{
		"3",
		"0","10","17","7","90",
		"6","735","5","516","4",
		"8","1","675","2","566"
	};
	auto route_2_lane_ids = std::vector<int> {
		-2,
		-2, -2, -2, 2, 2,
		2, 2, 2, 2, 2,
		-2, -2, -2, -2, -2
	};

	loop_road_ids = std::set<std::string> {
		"0","10","17","7","90",
		"6","735","5","516","4",
		"8","1","675","2", "566", "3"
	};

	loop_lane_ids = std::set<int> {
		-2, -2, -2, 2, 2,
		 2,  2,  2, 2, 2, 2, 
		-2, -2, -2, -2, -2, -2
	};

	auto empty_set = std::set<std::string>{};

	path_pub_timer = this->create_wall_timer(0.5s, std::bind(&PathPublisherNode::generatePaths, this));

	// Read map from file, using our path param
	std::string xodr_path = this->get_parameter("xodr_path").as_string();
	path_resolution = this->get_parameter("path_resolution").as_double();
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	map = new odr::OpenDriveMap(xodr_path, true, true, false, true);

	

	this->route1 = generate_path(route_1_road_ids, route_1_lane_ids, map, stop_road_ids);
	this->route2 = generate_path(route_2_road_ids, route_2_lane_ids, map, stop_road_ids);
	this->route2_nonstop = generate_path(route_2_road_ids, route_2_lane_ids, map, empty_set);
	this->path = this->route1;
}

void PathPublisherNode::generatePaths() {
	// Wait until odometry data is available
	if (cached_odom == nullptr) {
		RCLCPP_WARN(get_logger(), "Odometry not yet received, skipping...");
		return;
	}
	// Get current position and velocity
	Point current_pos = cached_odom->pose.pose.position;
	auto twist = cached_odom->twist.twist.linear;
	double speed = std::sqrt(twist.x*twist.x+twist.y*twist.y);

	paths_pub->publish(this->path);
	publish_paths_viz(this->path);
	RCLCPP_INFO(this->get_logger(), "publish path, speed %.2f", speed);


	auto currentLane = map->get_lane_from_xy_with_route(current_pos.x, current_pos.y, all_ids);
	if (currentLane == nullptr) {
		RCLCPP_WARN(get_logger(), "Lane could not be located given position (%.2f,%.2f) and map with %i roads", current_pos.x, current_pos.y, map->get_roads().size());
		if (speed < 0.5) {
			//we are hit a stop sign in an unkown lane, so go again
			this->path = this->route2_nonstop;
		}
		return;
	}
	auto currentRoadId = currentLane->road.lock()->id;
	
	RCLCPP_INFO(get_logger(), "Road %s, Current lane: %i", currentRoadId.c_str(), currentLane->id);
}