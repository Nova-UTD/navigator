/*
 * Package:   nova_pure_pursuit
 * Filename:  PurePursuitNode.hpp
 * Author:    Cristian Cruz
 * Email:     Cristian.CruzLopez@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
//#include "autoware_auto_msgs/msg/vehicle_control_command.hpp"

#include "nova_pure_pursuit/PurePursuit.hpp"

using namespace std::chrono_literals;


namespace Nova {
namespace PurePursuit {



class PurePursuitNode : public rclcpp::Node {

public:
  PurePursuitNode();
  ~PurePursuitNode();

 private:
  void update_lookahead_point();
  void timer_callback();
  void topic_callback(const std_msgs::msg::String::SharedPtr msg);

  std::unique_ptr<PurePursuit> controller;

  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;
    
  rclcpp::Publisher<std_msgs::msg::String>::
      SharedPtr steering_control_publisher;

  rclcpp::Subscription<std_msgs::msg::String>::
      SharedPtr trajectory_subscription;

}; // class PurePursuitNode


} // namespace PurePursuit
} // namespace Nova