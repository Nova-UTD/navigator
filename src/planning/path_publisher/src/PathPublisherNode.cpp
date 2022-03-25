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
using navigator::opendrive::get_centerline_as_xy;

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

voltron_msgs::msg::FinalPath PathPublisherNode::generate_path(std::vector<std::string> &road_ids, std::vector<int> &lane_ids, odr::OpenDriveMap *map, std::set<std::string> &stop_roads)
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
		odr::Line3D centerline = get_centerline_as_xy(*lane, lanesection->s0, lanesection->get_end(), step, lane_id>0);

		double speed = stop_roads.count(id) == 0 ? 5 : 0;
		for (odr::Vec3D point : centerline) {
			route.push_back(point);
			costed_path.speeds.push_back(speed);
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
	RCLCPP_INFO(this->get_logger(), "publish path, speed %.2f", speed);


	auto currentLane = map->get_lane_from_xy_with_route(current_pos.x, current_pos.y, all_ids);
	if (currentLane == nullptr) {
		RCLCPP_WARN(get_logger(), "Lane could not be located.");
		if (speed < 0.5) {
			//we are hit a stop sign in an unkown lane, so go again
			this->path = this->route2_nonstop;
		}
		return;
	}
	auto currentRoadId = currentLane->road.lock()->id;
	
	RCLCPP_INFO(get_logger(), "Road %s, Current lane: %i", currentRoadId.c_str(), currentLane->id);
	if (currentRoadId == "10" && this->path == this->route1) {
		RCLCPP_INFO(get_logger(), "SWITCHED PATH");
		this->path = this->route2;
	}
	if (this->path == this->route2 && speed < 0.5) {
		this->path = this->route2_nonstop; //disable the stop lane now that we have stopped.
		RCLCPP_INFO(get_logger(), "DONE STOPPING");
	}
	if (this->path == this->route2_nonstop && this->go_road_ids.count(currentRoadId) == 1 && speed > 3) {
		this-> path = this->route2; //we are past the stop lane, so we can enable it again
		RCLCPP_INFO(get_logger(), "ENABLE STOPPING");
	}
}
