#include "lidar_nudger/LidarNudgerNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LidarNudgerNode>());
	rclcpp::shutdown();
	return 0;
}