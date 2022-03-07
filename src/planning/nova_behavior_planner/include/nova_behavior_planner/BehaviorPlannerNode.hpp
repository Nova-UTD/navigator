#pragma once

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/final_path.hpp"
#include "voltron_msgs/msg/costed_paths.hpp"
#include "voltron_msgs/msg/costed_path.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nova_behavior_planner/BehaviorPlanner.hpp"
#include "nova_behavior_planner/BehaviorStates.hpp"

using namespace std::chrono_literals;
using FinalPath = voltron_msgs::msg::FinalPath;
using CostedPath = voltron_msgs::msg::CostedPath;
using CostedPaths = voltron_msgs::msg::CostedPaths;
using Odometry = nav_msgs::msg::Odometry;



namespace Nova {
namespace BehaviorPlanner {

constexpr auto message_frequency = 100ms; // will change
constexpr float STOP_SPEED = 0.0;
constexpr float YIELD_SPEED = 5.0;
constexpr float SLOW_SPEED = 10.0;


class BehaviorPlannerNode : public rclcpp::Node {

public:
    BehaviorPlannerNode();
    ~BehaviorPlannerNode();

private:  
    
    rclcpp::Publisher<FinalPath>::SharedPtr final_path_publisher;    
    rclcpp::Subscription<CostedPaths>::SharedPtr paths_subscription;
    rclcpp::Subscription<Odometry>::SharedPtr odometry_subscription;

    // var
    std::unique_ptr<BehaviorPlanner> behavior_planner;
    rclcpp::TimerBase::SharedPtr control_timer;
    
    CostedPaths costed_paths;
    Odometry current_position;
    FinalPath final_path;

    State current_state;    


    // functions
    void send_message();
    void update_paths(CostedPaths::SharedPtr ptr);
    void update_current_position(Odometry::SharedPtr ptr);
    void update_state();
    bool reached_desired_velocity(float desired_velocity);
  
};

}
}