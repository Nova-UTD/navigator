#include "nova_behavior_planner/BehaviorPlannerNode.hpp"
#include <chrono>
#include <memory>
#include <iostream>
#include <tf2/utils.h>
#include <cmath>
#include <unordered_set>

#include <boost/geometry/algorithms/within.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/algorithms/assign.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

using namespace Nova::BehaviorPlanner;

BehaviorPlannerNode::BehaviorPlannerNode() : rclcpp::Node("behavior_planner") {  
  std::string xodr_path = "data/maps/town07/Town07_Opt.xodr";
  this->current_state = LANEKEEPING;

  // xml parsing
  RCLCPP_INFO(this->get_logger(), "Reading from " + xodr_path);
  this->odr_map = new odr::OpenDriveMap(xodr_path, {true, true, true, false, true});

  this->control_timer = this->create_wall_timer
    (message_frequency, std::bind(&BehaviorPlannerNode::send_message, this));

  this->final_zone_publisher = this->create_publisher<ZoneArray>("zone_array", 10);

  this->odometry_subscription = this->create_subscription
    <Odometry>("/carla/odom", 8, std::bind(&BehaviorPlannerNode::update_current_speed, this, _1));

  this->path_subscription = this->create_subscription
    <FinalPath>("paths", 8, std::bind(&BehaviorPlannerNode::update_current_path, this, _1));
  
}

BehaviorPlannerNode::~BehaviorPlannerNode() {}

void BehaviorPlannerNode::update_current_path(FinalPath::SharedPtr ptr) {
  this->current_path = ptr;
}

void BehaviorPlannerNode::update_current_speed(Odometry::SharedPtr ptr) {
  this->current_position_x = ptr->pose.pose.position.x;
  this->current_position_y = ptr->pose.pose.position.y;
  tf2::Vector3 vector_velocity(ptr->twist.twist.linear.x, ptr->twist.twist.linear.y, ptr->twist.twist.linear.z);
  this->current_speed = std::sqrt(vector_velocity.dot(vector_velocity));
}

void BehaviorPlannerNode::send_message() {
  if (this->current_path == nullptr) return;
  
  // empty zone array and update state
  update_state();

  final_zone_publisher->publish(this->final_zones);
}

void BehaviorPlannerNode::update_state() {
  switch(current_state) {
    case LANEKEEPING:
      RCLCPP_INFO(this->get_logger(), "current state: LANEKEEPING");
      if(upcoming_intersection()) {
        current_state = STOPPING;
      }
      break;
    case STOPPING:
      RCLCPP_INFO(this->get_logger(), "current state: STOPPING");
      if (reached_desired_velocity(STOP_SPEED)) {
        current_state = STOPPED;
      }
      break;
    case STOPPED:
      RCLCPP_INFO(this->get_logger(), "current state: STOPPED");
      if (true) {
        if (final_zones.zones.size()) {
          final_zones.zones[0].max_speed = SLOW_SPEED;
        }
        current_state = IN_INTERSECTION;
      }
      break;
    case IN_INTERSECTION:
      RCLCPP_INFO(this->get_logger(), "current state: IN_INTERSECTION");
      RCLCPP_INFO(this->get_logger(), "array size" + std::to_string(final_zones.zones.size()));

      if(in_zone()) {
        reached_zone = true;
      }

      if (reached_zone) {
        if (!in_zone()) {
        RCLCPP_INFO(this->get_logger(), "OUT OF ZONE");
        // final_zones = {};
        current_state = TEMP;
        }
      }
      break;
    case TEMP:
      RCLCPP_INFO(this->get_logger(), "TEMP STATE TEMP STATE TEMP STATE");
      break;
  }
}


// used to determine if car is currently inside zone's polygon
bool BehaviorPlannerNode::in_zone() {

  if (!final_zones.zones.size()) {
    RCLCPP_INFO(this->get_logger(), "ERROR: IN_INTERSECTION state but no zones");
    return false;
  }

  // check if current position is in zone polygon
  typedef boost::geometry::model::d2::point_xy<double> point_type;
  typedef boost::geometry::model::polygon<point_type> polygon_type;
  
  polygon_type poly;
  point_type p(current_position_x, current_position_y);

  std::vector<point_type> points;
  for(auto point : final_zones.zones[0].poly.points) {
    points.push_back(point_type(point.x, point.y));
  }

  boost::geometry::assign_points(poly, points);
  return boost::geometry::within(p, poly);
}

// if path has intersection, then create zone for it
bool BehaviorPlannerNode::upcoming_intersection() {
  bool zones_made = false;
  std::unordered_set<std::string> junctions;
  final_zones.zones.clear();
  // find point on path closest to current location
  size_t closest_pt_idx = 0;
  float min_distance = -1;
  for (size_t i = 0; i < current_path->points.size(); i++) {
    float distance = pow(current_path->points[i].x - current_position_x, 2);
    distance += pow(current_path->points[i].y - current_position_y, 2); 
    if (min_distance == -1 || distance < min_distance) {
      min_distance = distance;
      closest_pt_idx = i;
    }
  }

  // each path-point spaced 25 cm apart, doing 400 pts gives us total coverage of 
  // 10000 cm or 100 meters
  size_t horizon_dist = 400;
  for(size_t offset = 0; offset < horizon_dist; offset++) {
    size_t i = (offset + closest_pt_idx) % current_path->points.size();
    // get road that current path point is in
    auto lane = navigator::opendrive::get_lane_from_xy(odr_map, current_path->points[i].x, current_path->points[i].y);
    //std::shared_ptr<const odr::Road> road = navigator::opendrive::get_road_from_xy(odr_map, current_path->points[i].x, current_path->points[i].y);
    if (lane == nullptr) {
        //RCLCPP_INFO(this->get_logger(), "(%f, %f): no road found for behavior planner", this->current_position_x, this->current_position_y);
        continue;
    }
    
    // check if road is a junction
    auto junction = lane->road.lock()->junction;
    auto id = lane->road.lock()->id;
    if (junction != "-1") {
      if (junctions.find(junction) != junctions.end()) {
          continue; //already seen this junction
      }
      //RCLCPP_INFO(this->get_logger(), "in junction " + junction + " for road " + id + " zones: %d", final_zones.zones.size());
      junctions.insert(junction);
      zones_made = true;
      Zone zone = navigator::zones_lib::to_zone_msg(odr_map->junctions[junction], odr_map);
      final_zones.zones.push_back(zone);
      // break;
    }
  }

  return zones_made;
}

// no perception data so just returning false for now
bool BehaviorPlannerNode::obstacles_present() {
  return false;
}

bool BehaviorPlannerNode::reached_desired_velocity(float desired_velocity) {

  tick += 1;
  float value = (int)(current_speed * 100);
  value /= 100;

  RCLCPP_INFO(this->get_logger(), "SPEED" + std::to_string(value));

  if (tick >= 10 && desired_velocity >= value) {
    tick = 0;
    return true;
  } else {
    return false;
  }

}