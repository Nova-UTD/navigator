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

using nav_msgs::msg::Odometry;
using sensor_msgs::msg::Imu;
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

using namespace std;
using namespace gtsam;

// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

class UnaryFactor : public NoiseModelFactor1<Pose2>
{
	// The factor will hold a measurement consisting of an (X,Y) location
	// We could this with a Point2 but here we just use two doubles
	double mx_, my_;

public:
	/// shorthand for a smart pointer to a factor
	typedef boost::shared_ptr<UnaryFactor> shared_ptr;

	// The constructor requires the variable key, the (X, Y) measurement value, and the noise model
	UnaryFactor(Key j, double x, double y, const SharedNoiseModel &model) : NoiseModelFactor1<Pose2>(model, j), mx_(x), my_(y) {}

	~UnaryFactor() override {}

	// Using the NoiseModelFactor1 base class there are two functions that must be overridden.
	// The first is the 'evaluateError' function. This function implements the desired measurement
	// function, returning a vector of errors when evaluated at the provided variable value. It
	// must also calculate the Jacobians for this measurement function, if requested.
	Vector evaluateError(const Pose2 &q, boost::optional<Matrix &> H = boost::none) const override
	{
		// The measurement function for a GPS-like measurement h(q) which predicts the measurement (m) is h(q) = q, q = [qx qy qtheta]
		// The error is then simply calculated as E(q) = h(q) - m:
		// error_x = q.x - mx
		// error_y = q.y - my
		// Node's orientation reflects in the Jacobian, in tangent space this is equal to the right-hand rule rotation matrix
		// H =  [ cos(q.theta)  -sin(q.theta) 0 ]
		//      [ sin(q.theta)   cos(q.theta) 0 ]
		const Rot2 &R = q.rotation();
		if (H)
			(*H) = (gtsam::Matrix(2, 3) << R.c(), -R.s(), 0.0, R.s(), R.c(), 0.0).finished();
		return (Vector(2) << q.x() - mx_, q.y() - my_).finished();
	}

	// The second is a 'clone' function that allows the factor to be copied. Under most
	// circumstances, the following code that employs the default copy constructor should
	// work fine.
	gtsam::NonlinearFactor::shared_ptr clone() const override
	{
		return boost::static_pointer_cast<gtsam::NonlinearFactor>(
			gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this)));
	}

	// Additionally, we encourage you the use of unit testing your custom factors,
	// (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
	// GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.
}; // UnaryFactor

OptimizationNode::OptimizationNode() : Node("optimization_node")
{
	// Create publishers and subscribers
	gnss_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/sensors/gnss/odom", 10,
																   [this](nav_msgs::msg::Odometry::SharedPtr msg)
																   { handleGNSS(msg); });
	imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("/sensors/zed/imu", 10,
																[this](sensor_msgs::msg::Imu::SharedPtr msg)
																{ handleIMU(msg); });

	optimization_timer_ = this->create_wall_timer(
		0.5s, std::bind(&OptimizationNode::doOptimization, this));

	isam_params.factorization = ISAM2Params::CHOLESKY;
	isam_params.relinearizeSkip = 10;

	isam_optimizer = boost::make_shared<ISAM2>();

	// Set IMU preintegration parameters
	double g = 9.8;
	auto w_coriolis = Vector3::Zero(); // zero vector
	Matrix33 measured_acc_cov =		   // TODO: Fix these vals!
		I_3x3 * pow(0.1, 2);
	Matrix33 measured_omega_cov =
		I_3x3 * pow(0.1, 2);
	// error committed in integrating position from velocities
	Matrix33 integration_error_cov =
		I_3x3 * pow(0.1, 2);

	auto imu_params = PreintegratedImuMeasurements::Params::MakeSharedU(g);
	imu_params->accelerometerCovariance =
		measured_acc_cov; // acc white noise in continuous
	imu_params->integrationCovariance =
		integration_error_cov; // integration uncertainty continuous
	imu_params->gyroscopeCovariance =
		measured_omega_cov; // gyro white noise in continuous
	imu_params->omegaCoriolis = w_coriolis;

	imu_bias = imuBias::ConstantBias(); // init with zero bias

	preintegrated_imu =
		boost::make_shared<PreintegratedImuMeasurements>(imu_params,
														 imu_bias);

	prev_imu_t = get_clock()->now().seconds() + get_clock()->now().nanoseconds() * 1e-9;
}

void OptimizationNode::handleGNSS(Odometry::SharedPtr msg)
{
	RCLCPP_INFO(get_logger(), "Received GNSS");
	// 1. Create a factor graph container and add factors to it

	// 2a. Add odometry factors
	// For simplicity, we will use the same noise model for each odometry factor
	if (node_idx > 1)
	{
		auto odometryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));
		// Create odometry (Between) factors between consecutive poses
		graph.emplace_shared<ImuFactor>(node_idx - 1, node_idx - 1,
										node_idx, node_idx, node_idx - 1,
										*preintegrated_imu);
	}

	// 2b. Add "GPS-like" measurements
	// We will use our custom UnaryFactor for this.
	auto unaryNoise =
		noiseModel::Diagonal::Sigmas(Vector2(0.1, 0.1)); // 10cm std on x,y
	auto pos = msg->pose.pose.position;
	graph.emplace_shared<UnaryFactor>(node_idx, pos.x, pos.y, unaryNoise);
	graph.print("\nFactor Graph:\n"); // print

	// 3. Create the data structure to hold the initialEstimate estimate to the solution
	// For illustrative purposes, these have been deliberately set to incorrect values
	Values initialEstimate;
	initialEstimate.insert(node_idx, Pose2(pos.x, pos.y, pos.z));
	initialEstimate.print("\nInitial Estimate:\n"); // print

	if (node_idx < 5)
	{
		RCLCPP_INFO(get_logger(), "Node immature, skipping");
		node_idx++;
		return;
	}

	// 4. Optimize using Levenberg-Marquardt optimization. The optimizer
	// accepts an optional set of configuration parameters, controlling
	// things like convergence criteria, the type of linear system solver
	// to use, and the amount of information displayed during optimization.
	// Here we will use the default set of parameters.  See the
	// documentation for the full set of parameters.
	isam_optimizer->update(graph, initialEstimate);

	// Reset our graph and values
	graph.resize(0);
	preintegrated_imu->resetIntegration();
	node_idx++;

	Values result = isam_optimizer->calculateEstimate();
	result.print("Final Result:\n");

	// 5. Calculate and print marginal covariances for all variables
	Marginals marginals(graph, result);
	cout << "x1 covariance:\n"
		 << marginals.marginalCovariance(1) << endl;
	cout << "x2 covariance:\n"
		 << marginals.marginalCovariance(2) << endl;
	cout << "x3 covariance:\n"
		 << marginals.marginalCovariance(3) << endl;
}

void OptimizationNode::handleIMU(Imu::SharedPtr msg)
{
	// find dt
	double new_t = (msg->header.stamp.sec +
					msg->header.stamp.nanosec * 1e-9); // Seconds
	double dt = new_t - prev_imu_t;
	prev_imu_t = new_t;

	if (dt <= 0)
	{
		RCLCPP_WARN(get_logger(), "IMU dt was <= 0, skipping");
		return;
	}

	// Integrate measurement
	auto acc = Vector3(
		msg->linear_acceleration.x,
		msg->linear_acceleration.y,
		msg->linear_acceleration.z);

	auto gyro = Vector3(
		msg->angular_velocity.x,
		msg->angular_velocity.y,
		msg->angular_velocity.z);

	preintegrated_imu->integrateMeasurement(
		acc, gyro, dt);
}

void OptimizationNode::doOptimization()
{
}
