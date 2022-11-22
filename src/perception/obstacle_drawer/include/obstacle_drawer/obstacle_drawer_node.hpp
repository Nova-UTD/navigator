/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_drawer_node.hpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#ifndef OBSTACLE_DRAWER_NODE_HPP
#define OBSTACLE_DRAWER_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include <memory>
#include <array>

namespace navigator {
namespace obstacle_drawer {

class ObstacleDrawer : public rclcpp::Node {
private:
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obstacle_marker_array_publisher;
    rclcpp::Subscription<voltron_msgs::msg::Obstacle3DArray>::SharedPtr obstacles_subscription;
    void draw_obstacles(const voltron_msgs::msg::Obstacle3DArray::SharedPtr msg) const;

public:
    ObstacleDrawer();
};
}
}

#endif