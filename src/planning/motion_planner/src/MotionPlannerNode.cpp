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

using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;
using voltron_msgs::msg::Trajectory;
using voltron_msgs::msg::TrajectoryPoint;

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
    RCLCPP_WARN(this->get_logger(), "ideal path has %d points", ideal_path->points.size());
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
 * @brief Smooths path velocities to achievable trapezoidal profiles and incorperates zone speed limits
 * 
 * Requires a trajectory to have more than 1 point to smooth.
 * 
 * @param trajectory - Trajectory to smooth
 * @param zones - Zones to set limiting speeds
 * @param max_accel - Maximum accelration, sets the up slope of the trapezoidal profile. m/s^2
 * @param max_decel - Maximum decelleration, sets the down-slope of a trapezoidal profile. 
 *    Certain unacheivable trajectories may result in this being exceeded. m/s^2
 * @param result_horizon - How many meters ahead to smooth the trajectory for. meters
 */
void MotionPlannerNode::smooth(Trajectory& trajectory, ZoneArray &zones, double max_accel,
  double max_decel, double result_horizon){

    if(trajectory.points.size() < 2) return;

  // For every segment (between points) in the trajectory, test against each zone to see if we
  // should add in a zone entry or exit point to the trajectory.
  
  // Build an linked list copy of the trajectory, since we will be inserting/removing points
  // from the middle frequently.
  std::list<TrajectoryPoint> t_points;
  t_points.push_back(trajectory.points[0]);
  double dist_traversed = 0;
  for (size_t i = 1; i < trajectory.points.size() && dist_traversed < result_horizon; i++) {
    TrajectoryPoint tp = trajectory.points[i];
    t_points.push_back(tp);

    TrajectoryPoint tp_prev = trajectory.points[i-1];

    // Ignore arclength for this approximation, looking further will not hurt
    double dx = tp.x - tp_prev.x;
    double dy = tp.y - tp_prev.y;
    dist_traversed += std::sqrt(dy*dy + dx*dx);
  }

  for(Zone z : zones.zones){
    boost_polygon zgon = to_boost_polygon(z);

    for(auto seg_end_it = std::next(t_points.begin()); seg_end_it != t_points.end(); seg_end_it++){
      auto seg_begin_it = std::prev(seg_end_it);

      boost_point seg_end{(*seg_end_it).x, (*seg_end_it).y};
      boost_point seg_begin{(*seg_begin_it).x, (*seg_begin_it).y};

      double &speed_end = (*seg_end_it).vx;
      double &speed_begin = (*seg_begin_it).vx;

      if(boost::geometry::within(seg_end, zgon)){
          speed_end = std::min(speed_end, (double)z.max_speed);
      }
        if(boost::geometry::within(seg_begin, zgon)){
          speed_end = std::min(speed_end, (double)z.max_speed);
      }

      boost::geometry::model::linestring<boost_point> segment{{seg_begin, seg_end}};

      // Check intersection with zone to see if we need to insert a point
      boost::geometry::model::multi_point<boost_point> intersections;
      if(boost::geometry::intersection(zgon, segment, intersections)){
        // set both endpoints to the floor of zone speed, 
        // as an easier version of inserting an new point at zone boundary
        for(boost_point ip : intersections){
          TrajectoryPoint tp;
          tp.x = ip.x();
          tp.y = ip.y();
          tp.vx = std::min({(double) z.max_speed, speed_end, speed_begin});
          t_points.insert(seg_end_it, tp);
        }
      }
    }
  }

  // Do a forward iteration of the list to rebuild the trajectory
  // and enforce acceleration profile. Uses physics formula for max speeds
  trajectory.points.clear();
  trajectory.points.push_back(*t_points.begin());
  for(auto seg_start = t_points.begin(), seg_end = std::next(t_points.begin());
    seg_end != t_points.end(); seg_start++, seg_end++){
    double dx = (*seg_end).x - (*seg_start).x; 
    double dy = (*seg_end).y - (*seg_start).y; 
    double dist = sqrt(dx*dx + dy*dy);

    double start_speed = (*seg_start).vx;
    double max_end_speed = std::sqrt(2*max_accel*dist+start_speed*start_speed);
    (*seg_end).vx = std::min((*seg_end).vx, max_end_speed);
    trajectory.points.push_back(*seg_end);
  }

  // Backward iteration of the list to enforce deceleration profile
  for(auto seg_start = std::prev(trajectory.points.end(),2), seg_end = std::prev(trajectory.points.end(),1);
    seg_end != trajectory.points.begin(); seg_start--, seg_end--){

    double dx = (*seg_end).x - (*seg_start).x; 
    double dy = (*seg_end).y - (*seg_start).y; 
    double dist = sqrt(dx*dx + dy*dy);

    double end_speed = (*seg_start).vx;
    double max_start_speed = std::sqrt(-2*max_decel*dist+end_speed*end_speed);
    (*seg_start).vx = std::min((*seg_start).vx, max_start_speed);
  }
}