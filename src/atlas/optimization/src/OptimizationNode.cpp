/**
 * @file OptimizationNode.cpp
 * @author Will Heitman
 * @brief ISAM2-based state estimator, based on GTSAM.
 * @version 0.1
 * @date 2022-04-05
 *
 * @copyright Nova at UT Dallas (c) 2022
 *
 * Inspired by: https://github.com/borglab/gtsam/blob/develop/examples/IMUKittiExampleGPS.cpp
 */

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

	current_bias = imuBias::ConstantBias(); // Init bias at zero

	noise_model_gps = noiseModel::Diagonal::Precisions(
		(Vector6() << Vector3::Constant(0), Vector3::Constant(1.0 / 0.07))
			.finished());

	// Set IMU preintegration parameters
	Matrix33 measured_acc_cov =
		I_3x3 * pow(0.01, 2);
	Matrix33 measured_omega_cov =
		I_3x3 * pow(0.01, 2); // TODO: Fix these temporary sdevs!! WSH.
	// error committed in integrating position from velocities
	Matrix33 integration_error_cov =
		I_3x3 * pow(0.01, 2);

	imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(9.8);
	auto w_coriolis = Vector3::Zero(); // zero vector
	imu_params->accelerometerCovariance =
		measured_acc_cov; // acc white noise in continuous
	imu_params->integrationCovariance =
		integration_error_cov; // integration uncertainty continuous
	imu_params->gyroscopeCovariance =
		measured_omega_cov; // gyro white noise in continuous
	imu_params->omegaCoriolis = w_coriolis;
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
	queued_imu_measurements.push_back(msg);
}

void OptimizationNode::doOptimization()
{
	RCLCPP_INFO(get_logger(), "HELLO");

	auto current_pose_key = X(current_key_idx);
	auto current_vel_key = V(current_key_idx);
	auto current_bias_key = B(current_key_idx);

	if (gnss_cached_ == nullptr)
	{
		RCLCPP_WARN(get_logger(), "No cached GNSS is available. Skipping optimization.");
		return;
	}

	if (!gps_added)
	{
	}

	// Try to add GNSS factor
	auto gnss_time = rclcpp::Time(gnss_cached_->header.stamp.sec, gnss_cached_->header.stamp.nanosec);
	auto now = rclcpp::Time(get_clock()->now().seconds(), get_clock()->now().nanoseconds());

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

	current_summarized_measurement =
		std::make_shared<PreintegratedImuMeasurements>(imu_params,
													   current_bias);

	// Loop through stored IMU frames, starting with the second frame.
	// This is because we need to extract dt.

	if (current_key_idx < 1)
	{
		current_key_idx++;
		return;
	}

	/**
	 * Process IMU measurements into factors
	 */
	auto previous_pose_key = X(current_key_idx - 1);
	auto previous_vel_key = V(current_key_idx - 1);
	auto previous_bias_key = B(current_key_idx - 1);
	for (int i = 1; i < queued_imu_measurements.size(); i++)
	{
		auto msg = queued_imu_measurements.at(i);

		auto acc = Vector3(
			msg->linear_acceleration.x,
			msg->linear_acceleration.y,
			msg->linear_acceleration.z);

		auto angular_vel = Vector3(
			msg->angular_velocity.x,
			msg->angular_velocity.y,
			msg->angular_velocity.z);

		auto prev_msg = queued_imu_measurements.at(i - 1);
		RCLCPP_INFO(get_logger(), "dt: %f", ((msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - (prev_msg->header.stamp.sec + prev_msg->header.stamp.nanosec * 1e-9)));
		double dt = ((msg->header.stamp.sec + msg->header.stamp.nanosec * 1e-9) - (prev_msg->header.stamp.sec + prev_msg->header.stamp.nanosec * 1e-9));

		current_summarized_measurement->integrateMeasurement(acc, angular_vel, dt);
	}

	factors.emplace_shared<ImuFactor>(
		previous_pose_key, previous_vel_key, current_pose_key,
		current_vel_key, previous_bias_key, *current_summarized_measurement);

	// Bias evolution as given in the IMU metadata
	auto sigma_between_b = noiseModel::Diagonal::Sigmas(
		(Vector6() << Vector3::Constant(
			 0.1), // TODO: Fix these two sigma values!
		 Vector3::Constant(0.1))
			.finished());
	factors.emplace_shared<BetweenFactor<imuBias::ConstantBias>>(
		previous_bias_key, current_bias_key, imuBias::ConstantBias(),
		sigma_between_b);

	// Add initial values for velocity and bias based on the previous
	// estimates
	values.insert(current_vel_key, current_vel);
	values.insert(current_bias_key, current_bias);

	if (factors.size() > 10)
	{
		isam.update(factors, values);
		RCLCPP_INFO(get_logger(), "ISAM matured, updating...");

		queued_imu_measurements.clear();
		factors.resize(0);
		values.clear();

		Values result = isam.calculateEstimate();
		current_pose = result.at<Pose3>(current_pose_key);
		current_vel = result.at<Vector3>(current_vel_key);
		current_bias = result.at<imuBias::ConstantBias>(current_bias_key);
	}
	else
	{
		RCLCPP_INFO(get_logger(), "ISAM size is now %i", isam.size());
	}

	current_key_idx++;
}
