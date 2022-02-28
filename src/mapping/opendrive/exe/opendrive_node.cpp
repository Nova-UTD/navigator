#include <memory> // std::make_shared
#include "rclcpp/rclcpp.hpp"
#include "opendrive/OpenDriveNode.hpp"

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OpenDriveNode>());
	rclcpp::shutdown();
	return 0;
}