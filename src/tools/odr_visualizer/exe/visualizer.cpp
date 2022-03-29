#include "odr_visualizer/OdrVisualizerNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OdrVisualizerNode>());
	rclcpp::shutdown();
	return 0;
}