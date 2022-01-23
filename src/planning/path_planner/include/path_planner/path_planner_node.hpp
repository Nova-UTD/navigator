/*
 * Package:   path_planner
 * Filename:  path_planner_node.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

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

#include <visualization_msgs/msg/marker_array.hpp>

#include <voltron_msgs/msg/route_costs.hpp>
#include <voltron_msgs/msg/costed_path.hpp>
#include <voltron_msgs/msg/costed_paths.hpp>

#include <path_planner/path_planner.hpp>

namespace navigator
{
    namespace path_planner
    {
        class PathPlannerNode : public rclcpp::Node
        {
        public:
            PathPlannerNode(const rclcpp::NodeOptions &node_options);
            
            // Timer callback to run logic and publish a message
            void execute();

            // Publish message to topic
            void publish_paths(std::vector<voltron_msgs::msg::CostedPath> paths);
            void publish_paths_viz(std::vector<voltron_msgs::msg::CostedPath> paths);


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

            // Runs the node every 100ms
            rclcpp::TimerBase::SharedPtr execute_timer;

            // Subscriptions
            rclcpp::Subscription<voltron_msgs::msg::RouteCosts>::SharedPtr route_costs_sub_ptr;
            rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr current_pose_sub_ptr;

            rclcpp::Publisher<voltron_msgs::msg::CostedPaths>::SharedPtr path_pub_ptr;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_ptr;

            // Store current between messages
            geometry_msgs::msg::PoseStamped current_pose;
            bool current_pose_init = false;

            voltron_msgs::msg::RouteCosts::SharedPtr current_route_costs;
            // no init flag needed as costs can be ignored

            // Needed for post to map transform
            tf2::BufferCore tf_buffer;
            tf2_ros::TransformListener tf_listener;

        };
    }
}

#endif