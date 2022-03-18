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
/*
PSEUDOCODE

On start:
	1. Read xodr file
	2. Iterate over each lane in each road
		a. For each lane, gather vertices and generate triangles
		b. Put tris in array.
	3. Convert tri array to triangle list Marker message (or MarkerArray?)

Every n seconds:
	1. Publish tri array

*/

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Vector3;
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

	// int roads[] = {20,875,21,630,3,0,10,17,7,90,6,735,5,516,4,8,1,765,2,566,3,0};
	onramp_sequence = std::vector<std::string>{
		"20","875","21","630"
	}
	loop_sequence = std::vector<std::string>{
		"3","0","10","17","7","90","6"
	};

	path_pub_timer = this->create_wall_timer(1s, std::bind(&PathPublisherNode::generatePaths, this));

	// Read map from file, using our path param
	std::string xodr_path = this->get_parameter("xodr_path").as_string();
	double draw_detail = this->get_parameter("draw_detail").as_double();
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	odr::OpenDriveMap odr(xodr_path, true, true, false, true);

}

void PathPublisherNode::generatePaths() {
	RCLCPP_INFO(get_logger(), "Publishing paths.");
	// marker_pub->publish(lane_markers);
}