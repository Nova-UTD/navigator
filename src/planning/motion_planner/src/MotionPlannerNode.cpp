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

#include <list>
#include <algorithm>

const double max_accel = 1.0;
const double max_decel = 1.0;

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
    zone_subscription = this->create_subscription<ZoneArray>("/planning/zone_array", 10, bind(&MotionPlannerNode::update_zones, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
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
    if(zones != nullptr){
      limit_to_zones(tmp, *zones);
    }
    smooth(tmp, max_accel, max_decel);
    trajectory_publisher->publish(tmp);
    return;
}

void MotionPlannerNode::update_path(voltron_msgs::msg::FinalPath::SharedPtr ptr) {
    ideal_path = ptr;
}

void MotionPlannerNode::update_zones(voltron_msgs::msg::ZoneArray::SharedPtr ptr)
{
    zones = ptr;
}



/*void MotionPlannerNode::update_steering_angle(voltron_msgs::msg::SteeringPosition::SharedPtr ptr) {
  steering_angle = ptr->data; //radians
}
*/
void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry = msg;
}
/*
//radians
double MotionPlannerNode::quat_to_heading(double x, double y, double z, double w) {
  //z component of euler angles
  double t3 = 2.0 * (w * z + x * y);
  double t4 = 1.0 - 2.0 * (y * y + z * z);
  return std::atan2(t3, t4) + M_PI;
}*/

/**
 * @brief Limits the speed of a trajectory to the mininmum speed of a zone.
 * 
 * Creates points at the boundary of each zone to ensure the speed limit is
 * recognized for the whole zone. Does not smooth speeds to ensure a physically
 * viable trajectory.
 * 
 * Operates in-place: trajectory will be modified.
 * 
 * @param trajectory 
 * @param zones 
 */
void MotionPlannerNode::limit_to_zones(Trajectory& trajectory, ZoneArray& zones){
    if(zones.zones.size() < 1u) return;
    if(trajectory.points.size() < 1u) return;

    // Copy into a list since we will be inserting multiple
    // points into the middle
    std::list<TrajectoryPoint> t_points;
    std::copy(trajectory.points.begin(), trajectory.points.end(), std::back_inserter(t_points));

    for(Zone z : zones.zones){
    boost_polygon zgon = to_boost_polygon(z);
    // Calculate first point specially, since for a trajectory with 1 point the loop code
    // will never be reached.
    boost_point first_point{(*t_points.begin()).x, (*t_points.begin()).y};
    if(boost::geometry::within(first_point, zgon)){
      (*t_points.begin()).vx = std::min((*t_points.begin()).vx, (double)z.max_speed);
    }

    // Iterate through each line segment, checking if the endpoint falls in any zones
    // and if the segment intersects with the border of any zones
    for(auto seg_end_it = std::next(t_points.begin()); seg_end_it != t_points.end(); seg_end_it++){
      auto seg_begin_it = std::prev(seg_end_it);

      boost_point seg_end{(*seg_end_it).x, (*seg_end_it).y};
      boost_point seg_begin{(*seg_begin_it).x, (*seg_begin_it).y};

      double &speed_end = (*seg_end_it).vx;
      double &speed_begin = (*seg_begin_it).vx;

      if(boost::geometry::within(seg_end, zgon)){
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
          if(tp != (*seg_end_it) && tp != (*seg_begin_it)){
            t_points.insert(seg_end_it, tp);
          }
        }
      }
    }
  }

  // Copy results back into trajectory
  trajectory.points.clear();
  std::copy(t_points.begin(), t_points.end(), std::back_inserter(trajectory.points));
}

/**
 * @brief Smooths path velocities to achievable trapezoidal profiles
 * 
 * Requires a trajectory to have more than 1 point for smoothing to occur.
 * 
 * @param trajectory - Trajectory to smooth
 * @param max_accel - Maximum accelration, sets the up slope of the trapezoidal profile. m/s^2
 * @param max_decel - Maximum decelleration, sets the down-slope of a trapezoidal profile. 
 *    Certain unacheivable (short) trajectories may result in this being exceeded. m/s^2
 */
void MotionPlannerNode::smooth(Trajectory& trajectory, double max_accel, double max_decel){

  if(trajectory.points.size() < 2u) return;

  // Do a forward iteration of the list to rebuild the trajectory
  // and enforce acceleration profile. Uses physics formula for max speeds
  for(int i = 0; i < trajectory.points.size()-1; i++){
    TrajectoryPoint &seg_end = trajectory.points[i+1];
    TrajectoryPoint &seg_begin = trajectory.points[i];
    double dx = seg_end.x - seg_begin.x; 
    double dy = seg_end.y - seg_begin.y; 
    double dist = sqrt(dx*dx + dy*dy);

    double start_speed = seg_begin.vx;
    double max_end_speed = std::sqrt(2*max_accel*dist+start_speed*start_speed);
    seg_end.vx = std::min(seg_end.vx, max_end_speed);
  }
  // Backward iteration of the list to enforce deceleration profile
  for(int i = trajectory.points.size()-1; i > 0; i--){
    TrajectoryPoint &seg_end = trajectory.points[i];
    TrajectoryPoint &seg_begin = trajectory.points[i-1];

    double dx = seg_end.x - seg_begin.x; 
    double dy = seg_end.y - seg_begin.y; 
    double dist = sqrt(dx*dx + dy*dy);

    double end_speed = seg_end.vx;
    double max_start_speed = std::sqrt(2*max_decel*dist+end_speed*end_speed);
    seg_begin.vx = std::min(seg_begin.vx, max_start_speed);
  }
}