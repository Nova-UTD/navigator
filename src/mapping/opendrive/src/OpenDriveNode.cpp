#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "opendrive/OpenDriveNode.hpp"

using namespace std::chrono_literals;

/*
PSEUDOCODE

On init: Load our HD map

On curb detections received (sub cb):
	1. Get latest map->base_link tf
	2. Find nearest lane segment to vehilce using tf
	3. Print ID of nearest segment
*/

OpenDriveNode::OpenDriveNode()
: Node("opendrive_node")
{
	RCLCPP_INFO(this->get_logger(), "Hello, world!");
}