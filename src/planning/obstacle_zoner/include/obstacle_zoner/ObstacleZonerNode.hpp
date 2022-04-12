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
    //extra space to put around obstacles to avoid near misses
    static constexpr double zone_padding = 2.0;
    //size of the slow zone that goes around around the padded stop zone
    static constexpr double slow_extent = 1.0;
    static constexpr double slow_speed = 2.0; //speed to set the slow zone two
    // pub/sub
    rclcpp::Publisher<ZoneArray>::SharedPtr zone_publisher;
    rclcpp::Subscription<Obstacle3DArray>::SharedPtr perception_subscription;

    void zone_perception(Obstacle3DArray::SharedPtr ptr);
    geometry_msgs::msg::Point32 extend_from_center(double extra_distance, double x, double y, double center_x, double center_y);

};

}
}