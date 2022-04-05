#pragma once

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/final_path.hpp"
#include "voltron_msgs/msg/costed_paths.hpp"
#include "voltron_msgs/msg/costed_path.hpp"
#include "voltron_msgs/msg/zone_array.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nova_behavior_planner/BehaviorPlanner.hpp"
#include "nova_behavior_planner/BehaviorStates.hpp"

// libOpenDRIVE stuff
#include "OpenDriveMap.h"
#include "pugixml/pugixml.hpp"
#include "Lanes.h"
#include "Road.h"
#include "Junction.h"
#include "opendrive_utils/OpenDriveUtils.hpp"
//#include "zone_lib/zone.hpp"

using namespace std::chrono_literals;
using FinalPath = voltron_msgs::msg::FinalPath;
using CostedPath = voltron_msgs::msg::CostedPath;
using CostedPaths = voltron_msgs::msg::CostedPaths;
using Odometry = nav_msgs::msg::Odometry;
using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;



namespace Nova {
namespace BehaviorPlanner {

constexpr auto message_frequency = 100ms; // will change
constexpr float STOP_SPEED = 0.0;
constexpr float YIELD_SPEED = 5.0;
constexpr float SLOW_SPEED = 10.0;
constexpr float SPEED_LIMIT = 20.0;


class BehaviorPlannerNode : public rclcpp::Node {

public:
    BehaviorPlannerNode();
    ~BehaviorPlannerNode();

private:  
    
    rclcpp::Publisher<ZoneArray>::SharedPtr final_zone_publisher;
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;
    rclcpp::Subscription<FinalPath>::SharedPtr path_subscription;


    // var
    rclcpp::TimerBase::SharedPtr control_timer;
	navigator::opendrive::types::OpenDriveMapPtr map;
    //std::shared_ptr<navigator::opendrive::types::MapInfo> map;

    ZoneArray final_zones;
    FinalPath::SharedPtr current_path;
    float current_speed;
    float current_position_x;
    float current_position_y;
    State current_state;

    // functions
    void send_message();
    void update_current_speed(Odometry::SharedPtr ptr);
    void update_current_path(FinalPath::SharedPtr ptr);
    void update_state();

    bool upcoming_intersection();
    bool obstacles_present();
    bool reached_desired_velocity(float desired_velocity);
  
    void add_stop_zone();

};

}
}