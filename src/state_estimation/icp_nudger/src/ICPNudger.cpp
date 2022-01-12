#include <chrono>
// #include <functional>
// #include <memory>
// #include <string>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <ros/time.h>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/convert.h>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <vector>
#include <Eigen/Core>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using namespace sensor_msgs::msg;
using namespace nav_msgs::msg;

/*
PSEUDOCODE

Subs for:
- /gps/odom
- /map/pcd
- /lidar/filtered
- /tf (special listener)

On PCD map, cache to pcl::PointCloud<pcl::PointXYZ>

Nudge if:
1. New GPS odom received, then
2. Grab next Lidar
3. Transform Lidar to /map and ICP with cached map
3. Set gps_received = false
4. Repeat

*/

class ICPNudger : public rclcpp::Node
{
	public:
		ICPNudger()
		: Node("icp_nudger")
		{
			this->declare_parameter<double>("nudge_period", 1.0); // Time between ICP updates, in seconds
			this->get_parameter<double>("nudge_period", nudge_period_);

			lidar_sub_ = this->create_subscription<PointCloud2>("/lidar", 1, //Only one message, too slow for queue!
				[this](PointCloud2::SharedPtr msg) { lidarCb(msg); }
			);
			map_sub_ = this->create_subscription<PointCloud2>("/map", 1,
				[this](PointCloud2::SharedPtr msg) { pcd_map_ = msg; } // Convert and cache
			);
			gps_sub_ = this->create_subscription<Odometry>("/gps", 1,
				[this](Odometry::SharedPtr msg) { cached_gps_ = msg; } // Convert and cache
			);

			aligned_lidar_pub_ = this->create_publisher<PointCloud2>("/lidar/nudged", 10);

			timer_ = this->create_wall_timer(std::chrono::duration<double>(nudge_period_), std::bind(&ICPNudger::tryICP, this));

			// TF2
			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
		}

	private:
		void lidarCb(PointCloud2::SharedPtr msg) {
			cached_lidar_ = msg;
		}

		void tryICP() {
			// RCLCPP_INFO(this->get_logger(), "Trying ICP...");

			// TF lidar to /map using GPS odom, convert to PCL PCD
			auto transformed_lidar = new PointCloud2();
			geometry_msgs::msg::TransformStamped gps_to_map;
			
			if(cached_lidar_ == nullptr || cached_gps_ == nullptr) {
				RCLCPP_WARN(this->get_logger(), "No Lidar or GPS data received. Skipping.");
				return;
			}
			
			auto gps_pose = cached_gps_->pose.pose;
			gps_to_map.transform.translation.x = gps_pose.position.x;
			gps_to_map.transform.translation.y = gps_pose.position.y;
			gps_to_map.transform.translation.z = gps_pose.position.z;
			gps_to_map.transform.rotation = gps_pose.orientation;

			gps_to_map.child_frame_id = "map";
			gps_to_map.header.frame_id = "base_link";
			gps_to_map.header.stamp = this->get_clock()->now();

			auto lidar_to_bl_tf = tf_buffer_->lookupTransform("base_link", cached_lidar_->header.frame_id.c_str(), this->get_clock()->now());

			tf2::doTransform(*cached_lidar_, *cached_lidar_, lidar_to_bl_tf);
			tf2::doTransform(*cached_lidar_, *transformed_lidar, gps_to_map);
			// RCLCPP_INFO(this->get_logger(), "Transform performed");

			pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_lidar(new pcl::PointCloud<pcl::PointXYZ>(5,1)); // Input to ICP, in PCL's pcd format
			pcl::fromROSMsg(*transformed_lidar, *pcl_lidar);
			pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_map(new pcl::PointCloud<pcl::PointXYZ>(5,1));; // Input to ICP, in PCL's pcd format
			pcl::fromROSMsg(*pcd_map_, *pcl_map);

			
			// Apply ICP https://pointclouds.org/documentation/tutorials/iterative_closest_point.html
			pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
			icp.setInputSource(pcl_lidar);
			icp.setInputTarget(pcl_map);

			pcl::PointCloud<pcl::PointXYZ> Final;
			icp.align(Final);

			std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
			// std::cout << icp.getFinalTransformation() << std::endl;

			Eigen::Affine3d eigen_affine_transform;
			eigen_affine_transform.matrix() = icp.getFinalTransformation().cast<double>();
			auto icp_tf_stamped = tf2::eigenToTransform(eigen_affine_transform);
			auto nudged_lidar = new PointCloud2();
			tf2::doTransform(*transformed_lidar, *nudged_lidar, icp_tf_stamped);
			tf2::doTransform(*nudged_lidar, *nudged_lidar, gps_to_map);

			aligned_lidar_pub_->publish(*nudged_lidar);

			RCLCPP_INFO_STREAM(this->get_logger(), "GPS TF: "<<
													gps_to_map.transform.translation.x<<", "
													<<gps_to_map.transform.translation.y<<", "
													<<gps_to_map.transform.translation.z<<" - "
													<<gps_to_map.transform.rotation.w<<", "
													<<gps_to_map.transform.rotation.x<<", "
													<<gps_to_map.transform.rotation.y<<", "
													<<gps_to_map.transform.rotation.z);

			RCLCPP_INFO_STREAM(this->get_logger(), "ICP TF: "<<
													icp_tf_stamped.transform.translation.x<<", "
													<<icp_tf_stamped.transform.translation.y<<", "
													<<icp_tf_stamped.transform.translation.z<<" - "
													<<icp_tf_stamped.transform.rotation.w<<", "
													<<icp_tf_stamped.transform.rotation.x<<", "
													<<icp_tf_stamped.transform.rotation.y<<", "
													<<icp_tf_stamped.transform.rotation.z);

			RCLCPP_INFO_STREAM(this->get_logger(), "Final TF: "<<
													icp_tf_stamped.transform.translation.x+gps_to_map.transform.translation.x<<", "
													<<icp_tf_stamped.transform.translation.y+gps_to_map.transform.translation.y<<", "
													<<icp_tf_stamped.transform.translation.z+gps_to_map.transform.translation.z<<" - "
													<<icp_tf_stamped.transform.rotation.w<<", "
													<<icp_tf_stamped.transform.rotation.x<<", "
													<<icp_tf_stamped.transform.rotation.y<<", "
													<<icp_tf_stamped.transform.rotation.z);
			// Generate PoseWithCovariance using fitness score, bias, and GPS odom

			// Publish nudged odom as Odometry msg
		}

		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
		rclcpp::Subscription<PointCloud2>::SharedPtr map_sub_;
		rclcpp::Subscription<PointCloud2>::SharedPtr lidar_sub_;
		rclcpp::Subscription<Odometry>::SharedPtr gps_sub_;
		rclcpp::Publisher<PointCloud2>::SharedPtr aligned_lidar_pub_;
		sensor_msgs::msg::PointCloud2::SharedPtr pcd_map_;
		sensor_msgs::msg::PointCloud2::SharedPtr cached_lidar_;
		Odometry::SharedPtr cached_gps_;
		double nudge_period_;
		rclcpp::TimerBase::SharedPtr timer_{nullptr};
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ICPNudger>());
	rclcpp::shutdown();
	return 0;
}