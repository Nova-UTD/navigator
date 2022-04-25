#include "obstacle_zoner/ObstacleZonerNode.hpp"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Dense"
#include <array>

using std::placeholders::_1;

using namespace navigator::obstacle_zoner;

ObstacleZonerNode::ObstacleZonerNode() : rclcpp::Node("obstacle_zoner") {  

  RCLCPP_INFO(this->get_logger(), "Start obstacle zoner");

  this->zone_publisher = this->create_publisher<ZoneArray>("obstacle_zone_array", 10);

  this->perception_subscription = this->create_subscription<Obstacle3DArray>("/sensors/zed/obstacle_array_3d", 8, std::bind(&ObstacleZonerNode::zone_perception, this, _1));
  this->tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  this->transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

ObstacleZonerNode::~ObstacleZonerNode() {}

void ObstacleZonerNode::update_tf() {
    try {
		currentTf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
	} catch (tf2::TransformException& e) {
		RCLCPP_INFO(this->get_logger(), "Could not transform base_link to map: %s", e.what());
		return;
	}
}

//generates two zones per object.
//  1. stop zone - centered on obstacle's bounding box center. size: bounding box size + zone_padding
//  2. slow zone - centered on obstacle's bounding box center. size: bounding box size + zone_padding + slow_zone
void ObstacleZonerNode::zone_perception(Obstacle3DArray::SharedPtr ptr) {
    ZoneArray zones;
    update_tf();
    double tf_x = this->currentTf.transform.translation.x;
    double tf_y = this->currentTf.transform.translation.y;
    Eigen::Quaternion quat(this->currentTf.transform.rotation.w, this->currentTf.transform.rotation.x, this->currentTf.transform.rotation.y, this->currentTf.transform.rotation.z);
    auto rot_mat = quat.toRotationMatrix();

    for (size_t i = 0; i < rot_mat.size(); i++) {
        // RCLCPP_INFO(this->get_logger(), std::to_string(rot_mat(i)));
    }

    for (const auto &obstacle : ptr->obstacles)
    {
        std::array<geometry_msgs::msg::Point, 8> corners;
        //transform corner
        for (size_t i = 0; i < obstacle.bounding_box.corners.size(); i++) {
            Eigen::Vector3d corner(obstacle.bounding_box.corners[i].x, obstacle.bounding_box.corners[i].y, 0);
            auto rotated_corner = rot_mat*corner;
            RCLCPP_INFO(this->get_logger(), std::to_string(obstacle.bounding_box.corners[i].x));
            corners[i].x = rotated_corner[0] + tf_x;
            corners[i].y = rotated_corner[1] + tf_y;
            RCLCPP_INFO(this->get_logger(), std::to_string(corners[i].x));
        }
        Zone stop_zone;
        stop_zone.max_speed = 0;
        stop_zone.cost = 999;

        //center is centroid of bounding box
        double center_x = 0;
        double center_y = 0;
        for (const auto& corner : obstacle.bounding_box.corners) {
            center_x += corner.x;
            center_y += corner.y;
        }
        center_x = center_x / 8;
        center_y = center_y / 8;
        //stop zone:
        //take x/y corner of the box. extend vector from center of box an extra zone_padding units. start at +/+ corner and go clockwise
        //+x, +y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, corners[0].x, corners[0].y, center_x, center_y));
        //+x, -y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, corners[1].x, corners[1].y, center_x, center_y));
        //-x, -y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, corners[2].x, corners[2].y, center_x, center_y));
        //-x, +y
        stop_zone.poly.points.push_back(extend_from_center(zone_padding, corners[3].x, corners[3].y, center_x, center_y));
        zones.zones.push_back(stop_zone);
        //slow zone:
        Zone slow_zone;
        slow_zone.max_speed = slow_speed;
        slow_zone.cost = 100;
        //+x, +y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, corners[0].x, corners[0].y, center_x, center_y));
        //+x, -y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, corners[1].x, corners[1].y, center_x, center_y));
        //-x, -y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, corners[2].x, corners[2].y, center_x, center_y));
        //-x, +y
        slow_zone.poly.points.push_back(extend_from_center(zone_padding+slow_extent, corners[3].x, corners[3].y, center_x, center_y));
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