#include "path_publisher/PathPublisherNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<PathPublisherNode>());
	rclcpp::shutdown();
	return 0;
}