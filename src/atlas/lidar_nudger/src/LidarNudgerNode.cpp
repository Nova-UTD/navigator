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

#include "lidar_nudger/LidarNudgerNode.hpp"
/*
PSEUDOCODE

On start:
	1. Read xodr file
	2. Init tf listener/buffer

Each time road points are received:
	1. Get mesh of current road
	2. Get all road-classified points from sem seg topic
	3. Transform points from base_link to map
	4. For each point
		a. Check if point falls within mesh. If not, add to outside_points vector
	5. Get center of mass for outside points and print. This is our alignment error.
*/

using sensor_msgs::msg::PointCloud2;
using namespace std::chrono_literals;

LidarNudgerNode::LidarNudgerNode() : Node("lidar_nudger_node") {

	srand (static_cast <unsigned> (time(0)));

	this->declare_parameter<std::string>("xodr_path", "/home/main/navigator/data/maps/town10/Town10HD_Opt.xodr");
	this->declare_parameter<double>("draw_detail", 1.0);

	road_cloud_sub = this->create_subscription<PointCloud2>("/road_cloud", 10,
						[this](PointCloud2::SharedPtr msg) { nudge(msg); }
					);

	// map_pub_timer = this->create_wall_timer(5s, std::bind(&LidarNudgerNode::publishMarkerArray, this));

	// Read map from file, using our path param
	std::string xodr_path = this->get_parameter("xodr_path").as_string();
	double draw_detail = this->get_parameter("draw_detail").as_double();
	RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
	odr::OpenDriveMap odr(xodr_path, true, true, false, true);

}

void LidarNudgerNode::nudge(PointCloud2::SharedPtr msg) {
	RCLCPP_INFO(get_logger(), "Received %i points to nudge", msg->data.size());
}