#include "zone_fusion/ZoneFusionNode.hpp"

#include <string.h>

using std::placeholders::_1;

using namespace navigator::zone_fusion;

ZoneFusionNode::ZoneFusionNode() : rclcpp::Node("zone_fusion") {  
    //create a subscription for each source topic
  this->sub_bp_zones = this->create_subscription<ZoneArray>("/planning/zone_array", 5, std::bind(&ZoneFusionNode::update_bp_zones, this, _1));
  this->sub_obstacle_zones = this->create_subscription<ZoneArray>("/planning/obstacle_zone_array", 5, std::bind(&ZoneFusionNode::update_obstacle_zones, this, _1));

  RCLCPP_INFO(this->get_logger(), "Start zone fuser");

  this->zone_publisher = this->create_publisher<ZoneArray>("zones", 10);
}

ZoneFusionNode::~ZoneFusionNode() {}

void ZoneFusionNode::update_bp_zones(ZoneArray::SharedPtr zones) {
    RCLCPP_INFO(this->get_logger(), "update bp");
    bp_zones = zones;
    send_message();
}

void ZoneFusionNode::update_obstacle_zones(ZoneArray::SharedPtr zones) {
    RCLCPP_INFO(this->get_logger(), "update obstacles");
    obstacle_zones = zones;
    send_message();
}


void ZoneFusionNode::send_message() {
    //combine all cached zones into a single message and publish
    ZoneArray dst;
    if (bp_zones != nullptr) {
        for (const auto& zone : bp_zones->zones) {
            dst.zones.push_back(zone);
        }
    }
    if (obstacle_zones != nullptr) {
        for (const auto& zone : obstacle_zones->zones) {
            dst.zones.push_back(zone);
        }
    }
    RCLCPP_INFO(this->get_logger(), "publish %d zones", dst.zones.size());
    this->zone_publisher->publish(dst);
}





