/*
 * Package:   lanelet2_global_planner_nodes
 * Filename:  lanelet2_global_planner_node.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <rclcpp/node_options.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <time_utils/time_utils.hpp>
#include <motion_common/motion_common.hpp>

#include <autoware_auto_msgs/msg/complex32.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <lanelet2_global_planner_nodes/lanelet2_global_planner_node.hpp>
#include <std_msgs/msg/string.hpp>
#include <common/types.hpp>

#include <chrono>
#include <cmath>
#include <string>
#include <memory>
#include <vector>

using namespace std::chrono_literals;
using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;
using autoware::common::types::TAU;
using autoware::planning::lanelet2_global_planner::Lanelet2GlobalPlanner;
using autoware::planning::lanelet2_global_planner::LaneRouteCosts;
using std::placeholders::_1;
using voltron_msgs::msg::RouteCost;
using voltron_msgs::msg::RouteCosts;

namespace autoware
{
  namespace planning
  {
    namespace lanelet2_global_planner_nodes
    {

      autoware_auto_msgs::msg::TrajectoryPoint convertToTrajectoryPoint(
          const geometry_msgs::msg::Pose &pose)
      {
        autoware_auto_msgs::msg::TrajectoryPoint pt;
        pt.x = static_cast<float>(pose.position.x);
        pt.y = static_cast<float>(pose.position.y);
        const auto angle = tf2::getYaw(pose.orientation);
        pt.heading = ::motion::motion_common::from_angle(angle);
        return pt;
      }

      Lanelet2GlobalPlannerNode::Lanelet2GlobalPlannerNode(
          const rclcpp::NodeOptions &node_options)
          : Node("lanelet2_global_planner_node", node_options),
            tf_listener(tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
      {
        current_pose_init = false;
        // Global planner instance init
        lanelet2_global_planner = std::make_shared<Lanelet2GlobalPlanner>();
        // Subcribers Goal Pose
        goal_pose_sub_ptr =
            this->create_subscription<geometry_msgs::msg::PoseStamped>(
                "goal_pose", rclcpp::QoS(10),
                std::bind(&Lanelet2GlobalPlannerNode::goal_pose_cb, this, _1));

        // Subcribers Current Pose
        current_pose_sub_ptr =
            this->create_subscription<autoware_auto_msgs::msg::VehicleKinematicState>(
                "vehicle_kinematic_state", rclcpp::QoS(10),
                std::bind(&Lanelet2GlobalPlannerNode::current_pose_cb, this, _1));

        // Global path publisher
        route_costs_pub_ptr = this->create_publisher<RouteCosts>("route_costs", rclcpp::QoS(10));

        // Update loop
        // TODO: pull timer period value from param file?
        update_loop_timer = this->create_wall_timer(
            1000ms, std::bind(&Lanelet2GlobalPlannerNode::update_loop_cb, this));

        // Create map client
        map_client = this->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Client");

        // Request binary map from the map loader node
        this->request_osm_binary_map();
      }

      void Lanelet2GlobalPlannerNode::request_osm_binary_map()
      {
        while (rclcpp::ok() && !map_client->wait_for_service(1s))
        {
          RCLCPP_WARN(this->get_logger(), "HAD map service not available yet. Waiting...");
        }
        if (!rclcpp::ok())
        {
          RCLCPP_ERROR(
              this->get_logger(),
              "Client interrupted while waiting for map service to appear. Exiting.");
        }

        auto request = std::make_shared<autoware_auto_msgs::srv::HADMapService_Request>();
        request->requested_primitives.push_back(
            autoware_auto_msgs::srv::HADMapService_Request::FULL_MAP);

        auto result = map_client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) !=
            rclcpp::executor::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(this->get_logger(), "Service call failed");
          throw std::runtime_error("Lanelet2GlobalPlannerNode: Map service call fail");
        }

        // copy message to map
        autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;

        // Convert binary map msg to lanelet2 map and set the map for global path planner
        lanelet2_global_planner->osm_map = std::make_shared<lanelet::LaneletMap>();
        autoware::common::had_map_utils::fromBinaryMsg(msg, lanelet2_global_planner->osm_map);

        // parse lanelet global path planner elements
        lanelet2_global_planner->parse_lanelet_element();
      }

      /**
       * Called every time the route planner needs to send a new message
       *
       */
      void Lanelet2GlobalPlannerNode::update_loop_cb()
      {
        if (!goal_pose_init)
        {
          RCLCPP_WARN(this->get_logger(), "Awaiting goal pose before setting route");
          return;
        }

        if (!current_pose_init)
        {
          RCLCPP_WARN(this->get_logger(), "Awaiting current pose before setting route");
          return;
        }

        LaneRouteCosts costs;
        auto start = convertToTrajectoryPoint(current_pose.pose);

        lanelet2_global_planner->fetch_routing_costs(start, costs);

        publish_route_costs(costs);
      }

      void Lanelet2GlobalPlannerNode::goal_pose_cb(
          const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {

        // transform and set the starting and goal point in the map frame
        goal_pose.header = msg->header;
        goal_pose.pose = msg->pose;
        geometry_msgs::msg::PoseStamped goal_pose_map = goal_pose;

        if (goal_pose.header.frame_id != "map")
        {
          if (!transform_pose_to_map(goal_pose, goal_pose_map))
          {
            // return: nothing happen
            return;
          }
          else
          {
            goal_pose = goal_pose_map;
          }
        }

        auto end = convertToTrajectoryPoint(goal_pose.pose);

        lanelet2_global_planner->set_destination(end);
        goal_pose_init = true;
      }

      void Lanelet2GlobalPlannerNode::current_pose_cb(
          const autoware_auto_msgs::msg::VehicleKinematicState::SharedPtr msg)
      {
        // convert msg to geometry_msgs::msg::Pose
        current_pose.pose.position.x = msg->state.x;
        current_pose.pose.position.y = msg->state.y;
        current_pose.pose.position.z = 0.0;
        current_pose.pose.orientation =
            motion::motion_common::to_quat<geometry_msgs::msg::Quaternion>(
                msg->state.heading);
        current_pose.header = msg->header;

        // transform to "map" frame if needed
        if (current_pose.header.frame_id != "map")
        {
          geometry_msgs::msg::PoseStamped current_pose_map = current_pose;

          if (transform_pose_to_map(current_pose, current_pose_map))
          {
            // transform ok: set current_pose to the pose in map
            current_pose = current_pose_map;
            current_pose_init = true;
          }
          else
          {
            // transform failed
            current_pose_init = false;
          }
        }
        else
        {
          // No transform required
          current_pose_init = true;
        }
      }

      void Lanelet2GlobalPlannerNode::publish_route_costs(LaneRouteCosts &costs)
      {
        RouteCosts route_msg;

        route_msg.header.set__frame_id("map");
        route_msg.header.set__stamp(now());

        // add costs to msg
        for (auto cost : costs)
        {
          RouteCost cost_msg;
          cost_msg.lane_id = cost.first;
          cost_msg.cost = cost.second;
          route_msg.costs.push_back(cost_msg);
        }

        route_costs_pub_ptr->publish(route_msg);
      }

      bool8_t Lanelet2GlobalPlannerNode::transform_pose_to_map(
          const geometry_msgs::msg::PoseStamped &pose_in,
          geometry_msgs::msg::PoseStamped &pose_out)
      {
        std::string source_frame = pose_in.header.frame_id;
        // lookup transform validity
        if (!tf_buffer.canTransform("map", source_frame, tf2::TimePointZero))
        {
          RCLCPP_ERROR(this->get_logger(), "Failed to transform Pose to map frame");
          return false;
        }

        // transform pose into map frame
        geometry_msgs::msg::TransformStamped tf_map;
        try
        {
          tf_map = tf_buffer.lookupTransform(
              "map", source_frame,
              time_utils::from_message(pose_in.header.stamp));
        }
        catch (const tf2::ExtrapolationException &)
        {
          // currently falls back to retrive newest transform available for availability,
          // Do validation of time stamp in the future
          tf_map = tf_buffer.lookupTransform("map", source_frame, tf2::TimePointZero);
        }

        // apply transform
        tf2::doTransform(pose_in, pose_out, tf_map);
        return true;
      }

    } // namespace lanelet2_global_planner_nodes
  }   // namespace planning
} // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
    autoware::planning::lanelet2_global_planner_nodes::Lanelet2GlobalPlannerNode)
