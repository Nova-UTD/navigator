/*
 * Package:   lanelet2_global_planner_nodes
 * Filename:  lanelet2_global_planner_node.hpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef LANELET2_GLOBAL_PLANNER_NODES__LANELET2_GLOBAL_PLANNER_NODE_HPP_
#define LANELET2_GLOBAL_PLANNER_NODES__LANELET2_GLOBAL_PLANNER_NODE_HPP_
// ros2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <time_utils/time_utils.hpp>

// autoware
#include <lanelet2_global_planner_nodes/visibility_control.hpp>
#include <lanelet2_global_planner/lanelet2_global_planner.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <common/types.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <voltron_msgs/msg/route_cost.hpp>
#include <voltron_msgs/msg/route_costs.hpp>
#include <had_map_utils/had_map_conversion.hpp>
#include <common/types.hpp>

// c++
#include <chrono>
#include <string>
#include <memory>
#include <vector>
#include <cmath>

using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float64_t;
using autoware::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;

namespace autoware
{
  namespace planning
  {
    namespace lanelet2_global_planner_nodes
    {
      class LANELET2_GLOBAL_PLANNER_NODES_PUBLIC Lanelet2GlobalPlannerNode : public rclcpp::Node
      {

      public:
        explicit Lanelet2GlobalPlannerNode(const rclcpp::NodeOptions &node_options);

        void request_osm_binary_map();
        void update_loop_cb();
        void goal_pose_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void current_pose_cb(const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg);
        void publish_route_costs(lanelet2_global_planner::LaneRouteCosts &costs);
        bool8_t transform_pose_to_map(
            const geometry_msgs::msg::PoseStamped &pose_in, geometry_msgs::msg::PoseStamped &pose_out);

      private:
        std::shared_ptr<Lanelet2GlobalPlanner> lanelet2_global_planner;
        rclcpp::Client<autoware_auto_msgs::srv::HADMapService>::SharedPtr map_client;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_ptr;
        rclcpp::Subscription<autoware_auto_msgs::msg::VehicleKinematicState>::SharedPtr
            current_pose_sub_ptr;
        rclcpp::Publisher<voltron_msgs::msg::RouteCosts>::SharedPtr route_costs_pub_ptr;
        rclcpp::TimerBase::SharedPtr update_loop_timer;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::PoseStamped goal_pose;
        bool8_t current_pose_init;
        bool8_t goal_pose_init;
        tf2::BufferCore tf_buffer;
        tf2_ros::TransformListener tf_listener;
      };
    } // namespace lanelet2_global_planner_nodes
  }   // namespace planning
} // namespace autoware

#endif // LANELET2_GLOBAL_PLANNER_NODES__LANELET2_GLOBAL_PLANNER_NODE_HPP_
