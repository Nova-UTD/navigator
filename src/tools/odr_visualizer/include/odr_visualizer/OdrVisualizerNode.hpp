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

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class OdrVisualizerNode : public rclcpp::Node {
public:
	OdrVisualizerNode();
	void publishMarkerArray();

private:
	visualization_msgs::msg::MarkerArray lane_markers;
	int point_count;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
    rclcpp::TimerBase::SharedPtr map_pub_timer{nullptr};
};