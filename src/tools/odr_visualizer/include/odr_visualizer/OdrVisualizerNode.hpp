#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "Lanes.h"
#include "Road.h"
#include "RefLine.h"
#include "opendrive_utils/OpenDriveUtils.hpp"

// Geometry stuff
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voltron_msgs/msg/polygon_array.hpp>

// Transforms
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// struct LaneBoostPoly {
// 	std::string id;
// 	typedef boost::geometry::model::polygon<point_type> poly;
// };

class OdrVisualizerNode : public rclcpp::Node
{
public:
	OdrVisualizerNode();
	void publishMarkerArray();
	void publishNearbyLanePolygons();

private:
	void generateMapMarkers();

	visualization_msgs::msg::MarkerArray lane_markers;
	int point_count;
	std::vector<navigator::opendrive::LaneIdentifier> nearby_lane_ids;
	navigator::opendrive::OpenDriveMapPtr odr_map;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	nav_msgs::msg::Odometry cached_odom_;
	rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	rclcpp::TimerBase::SharedPtr map_pub_timer{nullptr};
	rclcpp::TimerBase::SharedPtr check_surrounding_road_timer{nullptr};

	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};