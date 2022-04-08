#pragma once

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/zone_array.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "zone_lib/zone.hpp"

using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;
using Obstacle3DArray = voltron_msgs::msg::Obstacle3DArray;



namespace navigator {
namespace obstacle_zoner {


class ObstacleZonerNode : public rclcpp::Node {

public:
    ObstacleZonerNode();
    ~ObstacleZonerNode();

private:  

    // pub/sub
    rclcpp::Publisher<ZoneArray>::SharedPtr zone_publisher;
    rclcpp::Subscription<Obstacle3DArray>::SharedPtr perception_subscription;

    void zone_perception(Obstacle3DArray::SharedPtr ptr);

};

}
}