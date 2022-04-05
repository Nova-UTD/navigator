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

PathPublisherNode::PathPublisherNode() : Node("path_publisher_node") {

	std::string xodr_path = "data/maps/town07/Town07_Opt.xodr";
	paths_pub = this->create_publisher<FinalPath>("paths", 1);
	odom_sub = this->create_subscription<Odometry>("/odometry/filtered", 1, [this](Odometry::SharedPtr msg) {
		cached_odom = msg;
	});
	viz_pub = this->create_publisher<MarkerArray>("path_pub_viz", 1);
    
	auto route_1_road_ids = std::vector<std::string>{
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
	};

    for (auto s : route_1_road_ids) {
        all_ids.insert(s);
    }
	path_pub_timer = this->create_wall_timer(0.5s, std::bind(&PathPublisherNode::generatePaths, this));

	// Read map from file, using our path param
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	map = navigator::opendrive::load_map(xodr_path)->map;

	

	this->route1 = generate_path(route_1_road_ids, route_1_lane_ids, map);
	this->path = this->route1;
}

voltron_msgs::msg::FinalPath PathPublisherNode::generate_path(std::vector<std::string> &road_ids, std::vector<int> &lane_ids, navigator::opendrive::OpenDriveMapPtr map)
{
	std::vector<odr::Vec3D> route;
	FinalPath costed_path;
	double step = 0.25;
	for (size_t i = 0; i < road_ids.size(); i++) {
		std::string id = road_ids[i];
		int lane_id = lane_ids[i];
		auto road = map->roads[id];
		//there is only one lanesection per road on this map
		std::shared_ptr<odr::LaneSection> lanesection = *(road->get_lanesections().begin());
		odr::LaneSet laneset = lanesection->get_lanes();
		//RCLCPP_INFO(this->get_logger(), "There are %d lanes for road %s", laneset.size(), id.c_str());
		std::shared_ptr<odr::Lane> lane = nullptr;
        //loop through the laneset to find a pointer to the lane.
		for (auto l : laneset) {
			if (l->id == lane_id) {
				lane = l;
				break;
			}
		}
		if (lane == nullptr) {
			RCLCPP_WARN(this->get_logger(), "NO LANE FOR ROAD %s (i=%d)", id.c_str(), i);
			continue;
		}
		odr::Line3D centerline = navigator::opendrive::get_centerline_as_xy(*lane, lanesection->s0, lanesection->get_end(), step, lane_id>0);

		for (odr::Vec3D point : centerline) {
			route.push_back(point);
			costed_path.speeds.push_back(5);
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
	RCLCPP_INFO(this->get_logger(), "path viz");

	viz_pub->publish(marker_array);
}


void PathPublisherNode::generatePaths() {
	// Wait until odometry data is available
	if (cached_odom == nullptr) {
		RCLCPP_WARN(get_logger(), "Odometry not yet received, skipping...");
		return;
	}
	Point current_pos = cached_odom->pose.pose.position;

	auto twist = cached_odom->twist.twist.linear;
	double speed = std::sqrt(twist.x*twist.x+twist.y*twist.y);

	paths_pub->publish(this->path);
	publish_paths_viz(this->path);


	auto currentLane = navigator::opendrive::get_lane_from_xy_with_route(map, current_pos.x, current_pos.y, all_ids);
	if (currentLane == nullptr) {
		RCLCPP_WARN(get_logger(), "Lane could not be located.");
		return;
	}
	auto currentRoadId = currentLane->road.lock()->id;
	
	RCLCPP_INFO(get_logger(), "(%.2f, %.2f) Road %s, Current lane: %i", current_pos.x, current_pos.y, currentRoadId.c_str(), currentLane->id);
	//if (currentRoadId == "10" && this->path == this->route1) {
	//	RCLCPP_INFO(get_logger(), "SWITCHED PATH");
	//	this->path = this->route2;
	//}
}
