#include "obstacle_zoner/ObstacleZonerNode.hpp"

using std::placeholders::_1;

using namespace navigator::obstacle_zoner;

ObstacleZonerNode::ObstacleZonerNode() : rclcpp::Node("obstacle_zoner") {  

  RCLCPP_INFO(this->get_logger(), "Start obstacle zoner");

  this->zone_publisher = this->create_publisher<ZoneArray>("obstacle_zone_array", 10);

  this->perception_subscription = this->create_subscription
    <Obstacle3DArray>("/objects", 8, std::bind(&ObstacleZonerNode::zone_perception, this, _1));
}

ObstacleZonerNode::~ObstacleZonerNode() {}

void ObstacleZonerNode::zone_perception(Obstacle3DArray::SharedPtr ptr) {
    ZoneArray zones;
    for (const auto& obstacle : ptr->obstacles) {
        Zone zone;
        zone.max_speed = 0;
        zone.cost = 999;
        RCLCPP_INFO(this->get_logger(), "(%f,%f,%f)", obstacle.bounding_box.center.position.x, obstacle.bounding_box.center.position.y, obstacle.bounding_box.center.position.z);
        //take x/y corner of the box. start at +/+ corner and go clockwise
        //+x, +y
        geometry_msgs::msg::Point32 p;
        p.x = obstacle.bounding_box.center.position.x + obstacle.bounding_box.size.x;
        p.y = obstacle.bounding_box.center.position.y + obstacle.bounding_box.size.y;
        zone.poly.points.push_back(p);
        //+x, -y
        p.x = obstacle.bounding_box.center.position.x + obstacle.bounding_box.size.x;
        p.y = obstacle.bounding_box.center.position.y - obstacle.bounding_box.size.y;
        zone.poly.points.push_back(p);
        //-x, -y
        p.x = obstacle.bounding_box.center.position.x - obstacle.bounding_box.size.x;
        p.y = obstacle.bounding_box.center.position.y - obstacle.bounding_box.size.y;
        zone.poly.points.push_back(p);
        //-x, +y
        p.x = obstacle.bounding_box.center.position.x - obstacle.bounding_box.size.x;
        p.y = obstacle.bounding_box.center.position.y + obstacle.bounding_box.size.y;
        zone.poly.points.push_back(p);
        zones.zones.push_back(zone);
    }
    this->zone_publisher->publish(zones);
}



