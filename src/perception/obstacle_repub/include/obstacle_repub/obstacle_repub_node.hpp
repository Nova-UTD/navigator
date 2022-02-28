/*
 * Package:   obstacle_repub
 * Filename:  obstacle_repub_node.hpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef OBSTACLE_REPUB_NODE_HPP
#define OBSTACLE_REPUB_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "lgsvl_msgs/msg/detection3_d_array.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include <memory>


namespace navigator {
namespace obstacle_repub {

class ObstacleRepublisher : public rclcpp::Node {
private:
    rclcpp::Subscription<lgsvl_msgs::msg::Detection3DArray>::SharedPtr svl_obstacle_subscription;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr zed_obstacle_subscription;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr nova_obstacle_publisher;

    void svl_obstacle_callback(const lgsvl_msgs::msg::Detection3DArray::SharedPtr msg) const;
    void zed_obstacle_callback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) const;

public:
    ObstacleRepublisher();
};
}
}
#endif