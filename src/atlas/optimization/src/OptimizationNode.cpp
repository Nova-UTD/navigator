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

#include "OptimizationNode.hpp"
#include <boost/range/adaptor/reversed.hpp>
/*
FUNCTIONALITY

- Add a node every N seconds (update period)
	- For each supported sensor, add the appropriate factor if:
		- The factor is not stale
			- EX: If the update period is 0.5s but the GPS was
			  last received 0.7s ago, the GPS factor is stale and
			  should not be added.

- Integrate IMU data each time an update is received

*/

const int MAX_HISTORY = 100;
const auto UPDATE_PERIOD = 0.5s; // seconds

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::TransformStamped;
using geometry_msgs::msg::Vector3;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

OptimizationNode::OptimizationNode() : Node("optimization_node")
{
	gtsam::ISAM2Params isam_params;
	isam_params.factorization = gtsam::ISAM2Params::CHOLESKY;
	isam_params.relinearizeSkip = 10;
	isam = gtsam::ISAM2(isam_params);

	// Create publishers and subscribers
	gnss_sub_ = this->create_subscription<Odometry>("/sensors/gnss/odom", 10,
													[this](Odometry::SharedPtr msg)
													{ handleGNSS(msg); });
	imu_sub_ = this->create_subscription<Imu>("/sensors/zed/imu", 10,
											  [this](Imu::SharedPtr msg)
											  { handleIMU(msg); });

	optimization_timer_ = this->create_wall_timer(
		UPDATE_PERIOD, std::bind(&OptimizationNode::doOptimization, this));
}

void OptimizationNode::handleGNSS(Odometry::SharedPtr msg)
{
}

void OptimizationNode::handleIMU(Imu::SharedPtr msg)
{
	// RCLCPP_INFO(get_logger(), "Received IMU");

	// We won't have a cache if the node has just started
	if (imu_cached_ == nullptr)
	{
		RCLCPP_INFO(get_logger(), "No cached IMU. Setting now.");
		imu_cached_ = msg;
		return;
	}
	else
	{
		auto tic = rclcpp::Time(msg->header.stamp.sec, msg->header.stamp.nanosec);
		auto toc = rclcpp::Time(imu_cached_->header.stamp.sec, imu_cached_->header.stamp.nanosec);
		double dt = (tic - toc).nanoseconds();
		// RCLCPP_INFO(get_logger(), "%.2f ms", dt / 1e6);

		imu_cached_ = msg;
	}
}

void OptimizationNode::doOptimization()
{
	RCLCPP_INFO(get_logger(), "OPTIMIZING");
}
