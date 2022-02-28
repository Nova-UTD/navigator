#include <chrono>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

class OpenDriveNode : public rclcpp::Node
{
	public:
		OpenDriveNode();

	private:
};