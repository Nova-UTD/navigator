#pragma once

#include "rclcpp/rclcpp.hpp"
#include "voltron_msgs/msg/zone_array.hpp"
#include "voltron_msgs/msg/zone.hpp"
#include "zone_lib/zone.hpp"

#include <vector>

using ZoneArray = voltron_msgs::msg::ZoneArray;
using Zone = voltron_msgs::msg::Zone;



namespace navigator {
namespace zone_fusion {


class ZoneFusionNode : public rclcpp::Node {

public:
    ZoneFusionNode();
    ~ZoneFusionNode();

private:  

    // pub/sub
    rclcpp::Publisher<ZoneArray>::SharedPtr zone_publisher;
    rclcpp::Subscription<ZoneArray>::SharedPtr sub_bp_zones;
    rclcpp::Subscription<ZoneArray>::SharedPtr sub_obstacle_zones;

    ZoneArray::SharedPtr bp_zones;
    ZoneArray::SharedPtr obstacle_zones;

    void update_bp_zones(ZoneArray::SharedPtr zones);
    void update_obstacle_zones(ZoneArray::SharedPtr zones);
    void send_message();

};

}
}