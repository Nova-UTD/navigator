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
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/dataset.h>

class OptimizationNode : public rclcpp::Node
{
public:
	OptimizationNode();
	void publishMarkerArray();
	void publishNearbyLanePolygons();

private:
	void generateMapMarkers();

	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr gnss_sub_;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
	nav_msgs::msg::Odometry::SharedPtr gnss_cached_;
	sensor_msgs::msg::Imu::SharedPtr imu_cached_;
	void handleGNSS(nav_msgs::msg::Odometry::SharedPtr msg);
	void handleIMU(sensor_msgs::msg::Imu::SharedPtr msg);
	// rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub;
	// rclcpp::Publisher<voltron_msgs::msg::PolygonArray>::SharedPtr nearby_poly_pub;
	rclcpp::TimerBase::SharedPtr optimization_timer_{nullptr};

	void doOptimization();
	// rclcpp::TimerBase::SharedPtr check_surrounding_road_timer{nullptr};

	double imu_integrated_dx = 0.0;
	double imu_integrated_dy = 0.0;
	std::shared_ptr<gtsam::PreintegratedImuMeasurements> current_summarized_measurement = nullptr;

	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

	gtsam::ISAM2 isam;
};