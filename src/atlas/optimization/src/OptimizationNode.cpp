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

// using geometry_msgs::msg::Point;
using geometry_msgs::msg::Point32;
using geometry_msgs::msg::Polygon;
using geometry_msgs::msg::TransformStamped;
using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using std_msgs::msg::ColorRGBA;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

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

	current_key_idx = 0; // Tracks the key of the current node to be inserted

	noise_model_gps = noiseModel::Diagonal::Precisions(
		(Vector6() << Vector3::Constant(0), Vector3::Constant(1.0 / 0.07))
			.finished());
}

void OptimizationNode::handleGNSS(Odometry::SharedPtr msg)
{
	gnss_cached_ = msg;

	// GPSFactor gps_factor = GPSFactor(1, Point3)
	// auto gps_pose =
	//       Pose3(current_pose_global.rotation(), gps_measurements[i].position);
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

	auto current_pose_key = X(current_key_idx);
	auto current_vel_key = V(current_key_idx);
	auto current_bias_key = B(current_key_idx);

	// Try to add GNSS factor
	auto gnss_time = rclcpp::Time(gnss_cached_->header.stamp.sec, gnss_cached_->header.stamp.nanosec);
	auto now = get_clock()->now();
	if ((now - gnss_time) > 0.5s)
	{
		geometry_msgs::msg::Point gps_pos = gnss_cached_->pose.pose.position;
		geometry_msgs::msg::Quaternion gps_q = gnss_cached_->pose.pose.orientation;
		Point3 pos = Point3(gps_pos.x, gps_pos.y, gps_pos.z);
		Rot3 rot = Rot3(gps_q.w, gps_q.x, gps_q.y, gps_q.z); // https://gtsam.org/doxygen/a03336.html
		Pose3 gps_pose = Pose3(rot, pos);					 // https://gtsam.org/doxygen/a03288.html

		factors.emplace_shared<PriorFactor<Pose3>>(
			current_pose_key, gps_pose, noise_model_gps);
		values.insert(current_pose_key, gps_pose);
	}
	else
	{
		RCLCPP_WARN(get_logger(), "GNSS value is stale, skipping.");
	}

	// Add initial values for velocity and bias based on the previous
	// estimates
	// values.insert(current_vel_key, current_velocity);
	// values.insert(current_bias_key, current_bias);

	isam.update(factors, values);

	current_key_idx++;
}
