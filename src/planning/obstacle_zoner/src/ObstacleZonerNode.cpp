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
        double center_y = obstacle.bounding_box.center.position.y; //translate
        //stop zone:
        //take x/y corner of the box. extend vector from center of box an extra zone_padding units. start at +/+ corner and go clockwise
        //+x, +y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, obstacle.bounding_box.corners[0].x, obstacle.bounding_box.corners[0].y, center_x, center_y));
        //+x, -y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, obstacle.bounding_box.corners[1].x, obstacle.bounding_box.corners[1].y, center_x, center_y));
        //-x, -y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, obstacle.bounding_box.corners[2].x, obstacle.bounding_box.corners[2].y, center_x, center_y));
        //-x, +y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, obstacle.bounding_box.corners[3].x, obstacle.bounding_box.corners[3].y, center_x, center_y));
        zones.zones.push_back(stop_zone);
        //slow zone:
        Zone slow_zone;
        slow_zone.max_speed = slow_speed;
        slow_zone.cost = 100;
        //+x, +y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, obstacle.bounding_box.corners[0].x, obstacle.bounding_box.corners[0].y, center_x, center_y));
        //+x, -y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, obstacle.bounding_box.corners[1].x, obstacle.bounding_box.corners[1].y, center_x, center_y));
        //-x, -y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, obstacle.bounding_box.corners[2].x, obstacle.bounding_box.corners[2].y, center_x, center_y));
        //-x, +y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, obstacle.bounding_box.corners[3].x, obstacle.bounding_box.corners[3].y, center_x, center_y));
        zones.zones.push_back(stop_zone);
        zones.zones.push_back(slow_zone);

    }
    this->zone_publisher->publish(zones);
}

//returns the point distance ||(x,y)-center||+extra_distance from the center, that's in the same direction as (x,y) from the center
geometry_msgs::msg::Point32 ObstacleZonerNode::extend_from_center(double extra_distance, double x, double y, double center_x, double center_y) {
    double dx = x-center_x;
    double dy = y-center_y;
    double mag = sqrt(dx*dx+dy*dy);
    geometry_msgs::msg::Point32 p;
    p.x = center_x + dx/mag*(mag+extra_distance);
    p.y = center_y + dy/mag*(mag+extra_distance);
    return p;
}


