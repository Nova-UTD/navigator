#include "rrt/RRTNode.hpp"
#include <iostream>
#include <memory>
#include "rclcpp/rclcpp.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RRTNode>());
	rclcpp::shutdown();
	return 0;
}