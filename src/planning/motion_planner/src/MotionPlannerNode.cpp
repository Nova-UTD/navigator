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
const double max_lat_accel = 0.6;
const double max_decel = 1.0;

using namespace navigator::motion_planner;
using namespace navigator::zones_lib;

using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;
using voltron_msgs::msg::Trajectory;
using voltron_msgs::msg::TrajectoryPoint;

double dist_between_points(TrajectoryPoint& p1, TrajectoryPoint& p2) {
    return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}


MotionPlannerNode::MotionPlannerNode() : Node("motion_planner_node")
{
    trajectory_publisher = this->create_publisher<voltron_msgs::msg::Trajectory>("outgoing_trajectory", 8);
    path_subscription = this->create_subscription<voltron_msgs::msg::FinalPath>("/planning/paths", 10, bind(&MotionPlannerNode::update_path, this, std::placeholders::_1));
    zone_subscription = this->create_subscription<ZoneArray>("/planning/zones", 10, bind(&MotionPlannerNode::update_zones, this, std::placeholders::_1));
    odomtery_pose_subscription = this->create_subscription<nav_msgs::msg::Odometry>("/carla/odom", rclcpp::QoS(10),std::bind(&MotionPlannerNode::odometry_pose_cb, this, std::placeholders::_1));
    control_timer = this->create_wall_timer(message_frequency, bind(&MotionPlannerNode::send_message, this));
    
}

void MotionPlannerNode::send_message() {
    if (ideal_path == nullptr) {
        // RCLCPP_WARN(this->get_logger(), "motion planner has no input path, skipping...");
        return;
    }
    Trajectory tmp = build_trajectory(ideal_path, horizon);
    if(zones != nullptr){
      limit_to_zones(tmp, *zones);
    }
    limit_to_curvature(tmp, max_lat_accel);
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


void MotionPlannerNode::odometry_pose_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odometry = msg;
}


/**
 * @brief Looks for the point on the trajectory closest to the car, then adds all points less than horizon units away to trajectory
 *          (loops if at end of path)
 * 
 * @param horizon - number of points on the final path
 */
voltron_msgs::msg::Trajectory MotionPlannerNode::build_trajectory(voltron_msgs::msg::FinalPath::SharedPtr path, size_t horizon) {
    Trajectory t;
    //find closest point
    size_t min_idx = 0;
    double min_sqr_dist = std::numeric_limits<float>::max(); //big number
    double x = odometry->pose.pose.position.x;
    double y = odometry->pose.pose.position.y;
    for (size_t i = 0; i < path->points.size(); i++) {
        auto pt = path->points[i];
        double sqr_dist = (pt.x-x)*(pt.x-x)+(pt.y-y)*(pt.y-y); 
        if (sqr_dist < min_sqr_dist) {
            min_idx = i;
            min_sqr_dist = sqr_dist; 
        }
    }
    //add trimmed points
    for (size_t points = 0; points < horizon; points++) {
        size_t i = (min_idx + points) % ideal_path->points.size();
        TrajectoryPoint pt = voltron_msgs::msg::TrajectoryPoint();
        auto p = path->points[i];
        pt.x = p.x;
        pt.y = p.y;
        pt.vx = path->speeds[i];
        t.points.push_back(pt);
    }
    return t;
}

/**
 * @brief Limits the speed of the trajectory to the curvature, so that
 * the vehicle does not experience lateral acceleration greater than
 * the maximum comfortable acceleration.
 * 
 * @param trajectory
 * @param max_accel 
 * @param curvature_interval - the mininum distance interval to use for calculating curvature
 */
void MotionPlannerNode::limit_to_curvature(Trajectory &trajectory, double max_accel, double curvature_interval) {
    double interval_size = 0;
    auto interval_start = trajectory.points.begin();
    auto interval_end = trajectory.points.begin();

    // loop until the interval reaches end of trajectory
    while (interval_end != trajectory.points.end())
    {
      // Ensure that the interval is at least the size of the curvature interval
      while(interval_size < curvature_interval && std::next(interval_end) != trajectory.points.end())
      {
        interval_size += dist_between_points(*interval_end, *std::next(interval_end));
        interval_end++;
      }

      if(interval_size < curvature_interval)
      {
        break;
      }

      // Calculate the curvature of the interval, by finding the 
      // angle between the headings of the interval
      double s_dx = std::next(interval_start)->x - interval_start->x;
      double s_dy = std::next(interval_start)->y - interval_start->y;
      double e_dx = interval_end->x - std::prev(interval_end)->x;
      double e_dy = interval_end->y - std::prev(interval_end)->y;

      // Calculate angle between using dot product, so need to normalize first
      double s_mag = std::sqrt(s_dx * s_dx + s_dy * s_dy);
      double e_mag = std::sqrt(e_dx * e_dx + e_dy * e_dy);

      double dot = (s_dx * e_dx + s_dy * e_dy) / (s_mag * e_mag);
      double angle = std::acos(dot);

      // curvature is angle per meter
      double curvature = abs(angle / interval_size);
      if(curvature > 0){
        double max_speed = max_accel / curvature;
        // set each point in the interval no greater than max speed
        for(auto it = interval_start; it != interval_end; it++)
        {
         it->vx = std::min(it->vx, max_speed);
        }
      }

      // move the start of the interval by one point, and adjust the size
      interval_start++;
      interval_size -= dist_between_points(*interval_start, *std::prev(interval_start));
    }
}

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
    std::vector<double> speeds;
    std::vector<boost_polygon> polygons;
    //horizon distance for zone (each trajectory point is spaced 0.25m apart)
    constexpr double max_dist = horizon*0.25;
    boost_point car_point{odometry->pose.pose.position.x, odometry->pose.pose.position.y};
    // Copy into a list since we will be inserting multiple
    // points into the middle
    std::list<TrajectoryPoint> t_points;
    std::copy(trajectory.points.begin(), trajectory.points.end(), std::back_inserter(t_points));

    for(const Zone& zone : zones.zones){
        const boost_polygon zgon = navigator::zones_lib::to_boost_polygon(zone);
        if (boost::geometry::distance(car_point, zgon) > max_dist) {
            continue; //zone is outside horizon
        }
        const double max_speed = zone.max_speed;
        // Calculate first point specially, since for a trajectory with 1 point the loop code
        // will never be reached.
        boost_point first_point{(*t_points.begin()).x, (*t_points.begin()).y};
        if(boost::geometry::within(first_point, zgon)){
          (*t_points.begin()).vx = std::min((*t_points.begin()).vx, max_speed);
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
              speed_end = std::min(speed_end, max_speed);
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
            tp.vx = std::min({max_speed, speed_end, speed_begin});
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
  for(size_t i = 0; i < trajectory.points.size()-1; i++){
    TrajectoryPoint &seg_end = trajectory.points[i+1];
    TrajectoryPoint &seg_begin = trajectory.points[i];
    double dist = dist_between_points(seg_begin, seg_end);

    double start_speed = seg_begin.vx;
    double max_end_speed = std::sqrt(2*max_accel*dist+start_speed*start_speed);
    seg_end.vx = std::min(seg_end.vx, max_end_speed);
  }
  // Backward iteration of the list to enforce deceleration profile
  for(int i = trajectory.points.size()-1; i > 0; i--){
    TrajectoryPoint &seg_end = trajectory.points[i];
    TrajectoryPoint &seg_begin = trajectory.points[i-1];

    double dist = dist_between_points(seg_begin, seg_end);

    double end_speed = seg_end.vx;
    double max_start_speed = std::sqrt(2*max_decel*dist+end_speed*end_speed);
    seg_begin.vx = std::min(seg_begin.vx, max_start_speed);
  }
}