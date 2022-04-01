/*
 * Package:   MotionPlanner
 * Filename:  MotionPlanner.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Nova UTD
 * License:   MIT License
 */

//NOTE: LOOK IN HEADER FILE FOR COMMENTS ON FUNCTION DEFINITIONS

//#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include <boost/geometry.hpp>

#include "motion_planner/MotionPlannerNode.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "zone_lib/zone.hpp"


using namespace navigator::motion_planner;
using namespace navigator::zones_lib;

MotionPlannerNode::MotionPlannerNode() : Node("motion_planner_node")
{
    trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectory", 8);
    path_subscription = this->create_subscription<voltron_msgs::msg::FinalPath>("/planning/paths", 10, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    //odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    //current_pose_subscription = this->create_subscription<VehicleKinematicState>("vehicle_kinematic_state", rclcpp::QoS(10), std::bind(&MotionPlannerNode::current_pose_cb, this, std::placeholders::_1));
    //steering_angle_subscription = this->create_subscription<voltron_msgs::msg::SteeringPosition>("/can/steering_angle", 8, bind(&MotionPlannerNode::update_steering_angle, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
    //planner = std::make_shared<MotionPlanner>();
}

void MotionPlannerNode::send_message() {
    if (ideal_path == nullptr) {
        RCLCPP_WARN(this->get_logger(), "motion planner has no input path, skipping...");
        return;
    }
    auto tmp = voltron_msgs::msg::Trajectory();
    for (size_t i = 0; i < ideal_path->points.size(); i++) {
      auto t = voltron_msgs::msg::TrajectoryPoint();
      auto p = ideal_path->points[i];
      t.x = p.x;
      t.y = p.y;
      t.vx = ideal_path->speeds[i];
      tmp.points.push_back(t);
    }
    // smooth(tmp);
    trajectory_publisher->publish(tmp);
    return;
}

void MotionPlannerNode::update_path(voltron_msgs::msg::FinalPath::SharedPtr ptr) {
    ideal_path = ptr;
}


/*void MotionPlannerNode::update_steering_angle(voltron_msgs::msg::SteeringPosition::SharedPtr ptr) {
  steering_angle = ptr->data; //radians
}

void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  pose.heading = quat_to_heading(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  pose.x = msg->pose.pose.position.x;
  pose.y = msg->pose.pose.position.y;
  pose.xv = msg->twist.twist.linear.x;
  pose.yv = msg->twist.twist.linear.y;
}

//radians
double MotionPlannerNode::quat_to_heading(double x, double y, double z, double w) {
  //z component of euler angles
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(t3, t4) + M_PI;
}*/



/**
 * @brief Enforces trapezoidal velocity profile, prioritizing lower speeds.
 * 
 * Profile will never exceed accel_rate, but may decelerate faster
 * than decel_rate.
 * 
 * 
 * @param speeds: Velocity profile to smooth. 
 *  First and last points must be current and final velocities.
 *  Points must be unifority spaced.
 * @param accel_rate:
 *  Acceleration rate in m/s^2
 * @param decel_rate 
 *  Deceleration rate in m/s^2
 * @param spacing
 *  Constant arclength spacing between points.
 */
void _smooth(std::vector<double>& speeds, double accel_rate, double decel_rate, double spacing) {
    if (speeds.size() < 2) {
        return;
    }
    
    double curr_speed = speeds[0];
    // forward pass for acceleration smoothing
    for (size_t i = 1; i < speeds.size(); i++)
    {
      //this is from kinetic energy
      double max_speed = std::sqrt(2*accel_rate*spacing+curr_speed*curr_speed);

      speeds[i] = std::min(speeds[i], max_speed);;
      //update speed after this point
      curr_speed = speeds[i];
    }

    curr_speed = speeds[speeds.size()-1];
    // backward pass for deceleration smoothing
    for (int i = speeds.size()-2; i >= 0; i--)
    {
      //this is from kinetic energy. Going backwards, so deltaX = -spacing
      double max_speed = std::sqrt(-2*decel_rate*spacing+curr_speed*curr_speed);

      speeds[i] = std::min(speeds[i], max_speed);;
      //update speed after this point
      curr_speed = speeds[i];
    }
}

void MotionPlannerNode::smooth(voltron_msgs::msg::Trajectory& trajectory, voltron_msgs::msg::ZoneArray &zones){
  // push speeds to vector
  std::vector<double> speeds;
  for(size_t i = 0; i < trajectory.points.size(); i++) {
    speeds.push_back(trajectory.points[i].vx);
  }

  // Enforce speed limits for zones
  for(voltron_msgs::msg::Zone zone : zones.zones) {
    boost_polygon zone_region = to_boost_polygon(zone);
    for(size_t i = 0; i < speeds.size(); i++) {
      auto point = trajectory.points[i];
      boost_point bp = boost_point(point.x, point.y);
      // Todo: within(point, zone) or within(zone, point)?
      if(boost::geometry::within(bp, zone_region)){
        speeds[i] = std::min(speeds[i], double(zone.max_speed));
      }
    }
  }

  // TODO: I don't know these numbers (spacing, accel, decel) and just made them
  // up. Please replace with real numbers.
  // TODO: I was also told that the spacing is uniform, but I see
  // no guarantee that this is true in this file.
  _smooth(speeds, 5, 5, 1);

}