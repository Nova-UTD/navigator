#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>

// Message headers
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <voltron_msgs/msg/polygon_array.hpp>

// Transforms
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

// For finding elements in a vector
#include <algorithm>

// GTSAM
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Key.h>

// Misc.
#include <vector>

using namespace gtsam;

struct KittiCalibration
{
	double body_ptx;
	double body_pty;
	double body_ptz;
	double body_prx;
	double body_pry;
	double body_prz;
	double accelerometer_sigma;
	double gyroscope_sigma;
	double integration_sigma;
	double accelerometer_bias_sigma;
	double gyroscope_bias_sigma;
	double average_delta_t;
};

struct ImuMeasurement
{
	double time;
	double dt;
	Vector3 accelerometer;
	Vector3 gyroscope; // omega
};

struct GpsMeasurement
{
	double time;
	Vector3 position; // x,y,z
};

class OptimizationNode : public rclcpp::Node
{
public:
	OptimizationNode();
	void publishMarkerArray();
	void publishNearbyLanePolygons();

private:
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	void handleGNSS(nav_msgs::msg::Odometry::SharedPtr msg);
	void handleIMU(sensor_msgs::msg::Imu::SharedPtr msg);
	// rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	// rclcpp::Publisher<voltron_msgs::msg::PolygonArray>::SharedPtr nearby_poly_pub;
	rclcpp::TimerBase::SharedPtr optimization_timer_{nullptr};
	boost::shared_ptr<PreintegratedImuMeasurements> preintegrated_imu;
	imuBias::ConstantBias imu_bias;
	double prev_imu_t; // The timestamp of the last IMU reading, used to find dt

	// ISAM2 stuff
	NonlinearFactorGraph graph;
	ISAM2Params isam_params;

	boost::shared_ptr<ISAM2> isam_optimizer;
	int node_idx = 1;

	void doOptimization();
};