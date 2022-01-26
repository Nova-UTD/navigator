/*
 * Package:   path_planner
 * Filename:  path_planner_node.cpp
 * Author:    Egan Johnson
 * Email:     egan.johnson@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <autoware_auto_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_msgs/srv/had_map_service.hpp>
#include <autoware_auto_msgs/msg/vehicle_kinematic_state.hpp>
#include <voltron_msgs/msg/route_costs.hpp>
#include <voltron_msgs/msg/route_cost.hpp>
#include <voltron_msgs/msg/costed_paths.hpp>
#include <voltron_msgs/msg/costed_path.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <motion_common/motion_common.hpp>
#include <tf2/buffer_core.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include <path_planner/path_planner_node.hpp>
#include <path_planner/path_planner.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

using namespace navigator::path_planner;

using autoware_auto_msgs::msg::VehicleKinematicState;
using std::placeholders::_1;
using voltron_msgs::msg::CostedPath;
using voltron_msgs::msg::CostedPaths;
using voltron_msgs::msg::RouteCost;
using voltron_msgs::msg::RouteCosts;

PathPlannerNode::PathPlannerNode(const rclcpp::NodeOptions &node_options) : rclcpp::Node::Node("path_planner_node", node_options),
                                                                            tf_listener(tf_buffer, std::shared_ptr<rclcpp::Node>(this, [](auto) {}), false)
{
  path_planner = std::make_shared<PathPlanner>();

  // Placeholder route cost object. Needed to access current_route_costs->costs, even if empty
  current_route_costs = std::make_shared<RouteCosts>();

  // Give map to path planner
  map_client = this->create_client<autoware_auto_msgs::srv::HADMapService>("HAD_Map_Client");
  path_planner->set_map(request_osm_binary_map());

  // Set up subscriptions
  // Subcribers Routing costs
  route_costs_sub_ptr = this->create_subscription<RouteCosts>("route_costs", rclcpp::QoS(10), std::bind(&PathPlannerNode::route_costs_cb, this, _1));

  // Subcribers Current Pose
  current_pose_sub_ptr = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&PathPlannerNode::current_pose_cb, this, _1));

  // Create publisher
  path_pub_ptr = this->create_publisher<CostedPaths>("paths", rclcpp::QoS(10));

  // Publisher for vizualization in rviz2. This should eventually move to a different node
  viz_pub_ptr = this->create_publisher<visualization_msgs::msg::MarkerArray>("paths_viz", rclcpp::QoS(10));

  // Start execution loop. Todo: check if 300ms is a good number
  execute_timer = this->create_wall_timer(300ms, std::bind(&PathPlannerNode::execute, this));
}

/**
 * @brief The main loop for this node. Runs path planner logic and publishes messages.
 *
 * Called by execute_timer frequently
 */
void PathPlannerNode::execute()
{

  if (!current_pose_init)
  {
    RCLCPP_WARN(this->get_logger(), "Not publishing paths because no known location");
    return;
  }
  // don't need to worry about missing route data- paths will still generate

  // Call on path planner logic
  std::vector<CostedPath> results;
  path_planner->get_paths(current_pose.pose, current_route_costs->costs, results);

  // publish results
  publish_paths(results);
  publish_paths_viz(results);
}

/**
 * @brief Forms and publishes a CostedPaths message out of the provided paths
 *
 * @param paths
 */
void PathPlannerNode::publish_paths(std::vector<CostedPath> paths)
{
  CostedPaths path_msg;
  path_msg.header.stamp = this->now();
  path_msg.header.frame_id = "map";
  path_msg.paths = paths;

  path_pub_ptr->publish(path_msg);
}

// Tracking how many paths have been published lets us remove paths as well
int n_paths_published_viz = 0;

/**
 * @brief Forms and publishes a MarkerArray for paths
 *
 * @param paths
 */
void PathPlannerNode::publish_paths_viz(std::vector<CostedPath> paths)
{
  visualization_msgs::msg::MarkerArray marker_array;

  // Counter for the unique path ID and source of the bug
  int marker_id = 0;

  for (CostedPath path : paths)
  {
    visualization_msgs::msg::Marker marker;

    // Set header and identifiers
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "path_planner_viz";
    marker.id = marker_id;
    marker_id++;

    // Add data contents
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.points = path.points;
    
    // Set visual display. Not sure if this is needed
    marker.scale.x = 1;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    // Add path to array
    marker_array.markers.push_back(marker);
  }

  // delete extra paths from previous publishes
  int n_paths = marker_id;
  while(n_paths_published_viz > n_paths){
     visualization_msgs::msg::Marker marker;

    // Set header and identifiers
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "path_planner_viz";
    marker.id = marker_id;
    marker_id++;

    // Set to removing path
    marker.action = visualization_msgs::msg::Marker::DELETE;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;

    marker_array.markers.push_back(marker);

    // decrement counter
    n_paths_published_viz--;
  }
  n_paths_published_viz = n_paths;

  viz_pub_ptr->publish(marker_array);
}

/**
 * @brief Callback for route cost subscriber
 *
 * Pass updated costs to PathPlanner logic
 *
 * @param costs
 */
void PathPlannerNode::route_costs_cb(const RouteCosts::SharedPtr costs_msg)
{
  // TODO: validate header
  current_route_costs = costs_msg;
}

/**
 * @brief Callback for kinematic state subscriber
 *
 * Transform to TrajectoryPoint and pass to PathPlanner logic.
 * Copied from autoware code in Lanelet2GlobalPlannerNode
 *
 * @param state
 */
void PathPlannerNode::current_pose_cb(const VehicleKinematicState::SharedPtr state_msg)
{
  // TODO: validate header

  // convert msg to geometry_msgs::msg::Pose
  current_pose.pose.position.x = state_msg->state.x;
  current_pose.pose.position.y = state_msg->state.y;
  current_pose.pose.position.z = 0.0;
  current_pose.pose.orientation =
      motion::motion_common::to_quat<geometry_msgs::msg::Quaternion>(
          state_msg->state.heading);
  current_pose.header = state_msg->header;

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
      // transform failed, don't want to use untransformed pose
      current_pose_init = false;
    }
  }
  else
  {
    // No transform required
    current_pose_init = true;
  }
}

/**
 * @brief Requests and returns the current lanelet map.
 *
 * Needs the map client to be initialized first.
 * Copied from autoware code in Lanelet2GlobalPlannerNode
 *
 * @return lanelet::LaneletMapPtr
 */
lanelet::LaneletMapPtr PathPlannerNode::request_osm_binary_map()
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
    throw std::runtime_error("PathPlannerNode: Map service call fail");
  }

  // copy message to map
  autoware_auto_msgs::msg::HADMapBin msg = result.get()->map;

  // Convert binary map msg to lanelet2 map and set the map for global path planner
  auto osm_map = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(msg, osm_map);
  return osm_map;
}

/**
 * Copied from Lanelet2GlobalPlannerNode autoware code
 *
 * @param pose_in
 * @param pose_out
 * @return true
 * @return false
 */
bool PathPlannerNode::transform_pose_to_map(
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

RCLCPP_COMPONENTS_REGISTER_NODE(navigator::path_planner::PathPlannerNode)