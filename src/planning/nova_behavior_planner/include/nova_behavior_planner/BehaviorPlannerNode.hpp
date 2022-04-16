#pragma once

#include <array>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/final_path.hpp"
#include "voltron_msgs/msg/costed_paths.hpp"
#include "voltron_msgs/msg/costed_path.hpp"
#include "voltron_msgs/msg/zone_array.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/bounding_box3_d.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "nova_behavior_planner/BehaviorPlanner.hpp"
#include "nova_behavior_planner/BehaviorStates.hpp"

#include "tf2/convert.h"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "pugixml/pugixml.hpp"
#include "Lanes.h"
#include "Road.h"
#include "Junction.h"
#include "opendrive_utils/OpenDriveUtils.hpp"
#include "zone_lib/zone.hpp"

using namespace std::chrono_literals;
using FinalPath = voltron_msgs::msg::FinalPath;
using CostedPath = voltron_msgs::msg::CostedPath;
using CostedPaths = voltron_msgs::msg::CostedPaths;
using Odometry = nav_msgs::msg::Odometry;
using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;
using BoundingBox = voltron_msgs::msg::BoundingBox3D;

// temp perception
using Obstacles = voltron_msgs::msg::Obstacle3DArray;
using Obstacle = voltron_msgs::msg::Obstacle3D;

// Boost
using Point = geometry_msgs::msg::Point;
typedef boost::geometry::model::d2::point_xy<double> point_type;
typedef boost::geometry::model::polygon<point_type> polygon_type;

namespace Nova {
namespace BehaviorPlanner {

constexpr auto message_frequency = 100ms;
constexpr float STOP_SPEED = 0.0;
constexpr float YIELD_SPEED = 3.0;
constexpr float SPEED_LIMIT = 10.0;

class BehaviorPlannerNode : public rclcpp::Node {

public:
    BehaviorPlannerNode();
    ~BehaviorPlannerNode();

private:  
    enum class SignalType {
        //order from least restrictive to most restrictive
        None=0,
        Yield=1,
        Stop=2,
    };

    // pub/sub
    rclcpp::Publisher<ZoneArray>::SharedPtr final_zone_publisher;
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;
    rclcpp::Subscription<FinalPath>::SharedPtr path_subscription;
    rclcpp::Subscription<Obstacles>::SharedPtr obstacles_subscription;
    
    rclcpp::TimerBase::SharedPtr control_timer;
    FinalPath::SharedPtr current_path;
    Obstacles::SharedPtr current_obstacles;
    ZoneArray final_zones;

    // odr
	navigator::opendrive::types::OpenDriveMapPtr map;
    std::shared_ptr<navigator::opendrive::types::MapInfo> map_info;

    // need to transform bounding boxes to world space
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    geometry_msgs::msg::TransformStamped currentTf;

    // stopping variables
    float current_speed;
    float current_position_x;
    float prev_position_x;
    float current_position_y;
    float prev_position_y;
    
    // state variables
    State current_state;
    bool reached_zone;
    int stop_ticks;
    int yield_ticks;
    std::vector<int> obs_with_ROW; // holds IDs of obstacles with right-of-way

    // pub/sub functions
    void update_current_speed(Odometry::SharedPtr ptr);
    void update_current_path(FinalPath::SharedPtr ptr);
    void update_current_obstacles(Obstacles::SharedPtr ptr);
    void update_tf();
    std::array<geometry_msgs::msg::Point, 8> transform_obstacle(const Obstacle& obstacle);
    void send_message();
    
    // FSM
    void update_state();

    // transition functions
    float zone_point_distance(float x, float y);
    bool point_in_zone(float x, float y);
    bool poly_in_zone(BoundingBox obs_bbox);
    bool obstacles_present(bool in_junction = false);
    void check_right_of_way();
    bool reached_desired_velocity(float desired_velocity);
    bool is_stopped();
    bool upcoming_intersection();
    SignalType classify_signal(const navigator::opendrive::Signal& signal);


};

}
}