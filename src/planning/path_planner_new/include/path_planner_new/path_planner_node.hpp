#ifndef PATH_PLANNER__PATH_PLANNER_NODE_HPP_
#define PATH_PLANNER__PATH_PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <had_map_utils/had_map_conversion.hpp>

#include <voltron_msgs/msg/route_costs.hpp>

#include <path_planner_new/path_planner.hpp>

namespace navigator
{
    namespace path_planner
    {
        class PathPlannerNode : public rclcpp::Node
        {
        public:
            PathPlannerNode(const rclcpp::NodeOptions &node_options);
            
            // Subscription callbacks
            void route_costs_cb(const voltron_msgs::msg::RouteCosts::SharedPtr msg);
            void current_pose_cb(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg);

            // Get lanelet map from service
            lanelet::LaneletMapPtr request_osm_binary_map();

            // Helper for current_pose_cb
            bool transform_pose_to_map(const geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::PoseStamped &pose_out);


        private:
            std::shared_ptr<PathPlanner> path_planner;

            rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client;

            // Subscriptions
            rclcpp::Subscription<voltron_msgs::msg::RouteCosts>::SharedPtr route_costs_sub_ptr;
            rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr current_pose_sub_ptr;

            // Store current pose between messages
            geometry_msgs::msg::PoseStamped current_pose;
            bool current_pose_init = false;

            // Needed for post to map transform
            tf2::BufferCore tf_buffer;
            tf2_ros::TransformListener tf_listener;

        };
    }
}

#endif