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

//generates two zones per object.
//  1. stop zone - centered on obstacle's bounding box center. size: bounding box size + zone_padding
//  2. slow zone - centered on obstacle's bounding box center. size: bounding box size + zone_padding + slow_zone
void ObstacleZonerNode::zone_perception(Obstacle3DArray::SharedPtr ptr) {
    ZoneArray zones;
    for (const auto& obstacle : ptr->obstacles) {
        Zone stop_zone;
        stop_zone.max_speed = 0;
        stop_zone.cost = 999;
        double center_x = obstacle.bounding_box.center.position.x;
        double center_y = -obstacle.bounding_box.center.position.y; //translate
        double extent_x = obstacle.bounding_box.size.x + zone_padding;
        double extent_y = obstacle.bounding_box.size.x + zone_padding;
        //stop zone:
        //take x/y corner of the box. start at +/+ corner and go clockwise
        //+x, +y
        geometry_msgs::msg::Point32 p;
        p.x = obstacle.bounding_box.corners[0].x;
        p.y = obstacle.bounding_box.corner[0].y
        stop_zone.poly.points.push_back(p);
        //+x, -y
        p.x = obstacle.bounding_box.corners[1].x;
        p.y = obstacle.bounding_box.corner[1].y
        stop_zone.poly.points.push_back(p);
        //-x, -y
        p.x = obstacle.bounding_box.corners[2].x;
        p.y = obstacle.bounding_box.corner[2].y
        stop_zone.poly.points.push_back(p);
        //-x, +y
        p.x = obstacle.bounding_box.corners[3].x;
        p.y = obstacle.bounding_box.corner[3].y
        stop_zone.poly.points.push_back(p);
        zones.zones.push_back(stop_zone);
        //slow zone:
        Zone slow_zone;
        slow_zone.max_speed = slow_speed;
        slow_zone.cost = 100;
        double slow_extent_x = extent_x + slow_extent;
        double slow_extent_y = extent_y + slow_extent;
        p.x = center_x + slow_extent_x;
        p.y = center_y + slow_extent_y;
        slow_zone.poly.points.push_back(p);
        //+x, -y
        p.x = center_x + slow_extent_x;
        p.y = center_y - slow_extent_y;
        slow_zone.poly.points.push_back(p);
        //-x, -y
        p.x = center_x - slow_extent_x;
        p.y = center_y - slow_extent_y;
        slow_zone.poly.points.push_back(p);
        //-x, +y
        p.x = center_x - slow_extent_x;
        p.y = center_y + slow_extent_y;
        slow_zone.poly.points.push_back(p);
        zones.zones.push_back(slow_zone);

    }
    this->zone_publisher->publish(zones);
}



