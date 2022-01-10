/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_drawer.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "voltron_msgs/msg/bounding_box3_d.hpp"
#include <memory>
#include <chrono>
#include <array>

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace navigator {
namespace obstacle_drawer {

class ObstacleDrawer : public rclcpp::Node {

public:
    ObstacleDrawer() : Node("obstacle_drawer_node") {
        this->vehicle_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/obstacles/visualizations/vehicles", 10);
        this->vehicle_subscription = this->create_subscription<voltron_msgs::msg::Obstacle3DArray>(
         "/obstacles/vehicles", 10, std::bind(&ObstacleDrawer::vehicle_array_draw, this, _1));

        this->pedestrian_marker_publisher = this->create_publisher<visualization_msgs::msg::Marker>("/obstacles/visualizations/pedestrians", 10);
        this->pedestrian_subscription = this->create_subscription<voltron_msgs::msg::Obstacle3DArray>(
         "/obstacles/pedestrians", 10, std::bind(&ObstacleDrawer::pedestrian_array_draw, this, _1));
    }

private:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vehicle_marker_publisher;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pedestrian_marker_publisher;
    rclcpp::Subscription<voltron_msgs::msg::Obstacle3DArray>::SharedPtr vehicle_subscription;
    rclcpp::Subscription<voltron_msgs::msg::Obstacle3DArray>::SharedPtr pedestrian_subscription;

    void vehicle_array_draw(const voltron_msgs::msg::Obstacle3DArray::SharedPtr msg) const {
        
        // setting up marker settings
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "base_link";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(250ms);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.scale.x = 0.2;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.header.frame_id = "base_link";

        for (const auto obstacles : msg->obstacles) {
            draw_3d_box(obstacles.bounding_box.corners, marker);
        }

        this->vehicle_marker_publisher->publish(marker);
    }

    void pedestrian_array_draw(const voltron_msgs::msg::Obstacle3DArray::SharedPtr msg) const {
        
        // setting up marker settings
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "base_link";
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.lifetime = rclcpp::Duration(250ms);
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        marker.scale.x = 0.1;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        for (const auto obstacles : msg->obstacles) {            
            draw_3d_box(obstacles.bounding_box.corners, marker);
        }

        this->pedestrian_marker_publisher->publish(marker);
    }

    // this will push line segments (two points) into the marker
    void draw_3d_box(const std::array<geometry_msgs::msg::Point, 8UL>& corners, visualization_msgs::msg::Marker& marker) const
    {
        marker.points.push_back(corners[0]);
        marker.points.push_back(corners[1]);
        marker.points.push_back(corners[0]);
        marker.points.push_back(corners[3]);
        marker.points.push_back(corners[0]);
        marker.points.push_back(corners[4]);
        marker.points.push_back(corners[2]);
        marker.points.push_back(corners[1]);
        marker.points.push_back(corners[2]);
        marker.points.push_back(corners[3]);
        marker.points.push_back(corners[2]);
        marker.points.push_back(corners[6]);
        marker.points.push_back(corners[5]);
        marker.points.push_back(corners[1]);
        marker.points.push_back(corners[5]);
        marker.points.push_back(corners[4]);
        marker.points.push_back(corners[5]);
        marker.points.push_back(corners[6]);
        marker.points.push_back(corners[7]);
        marker.points.push_back(corners[3]);
        marker.points.push_back(corners[7]);
        marker.points.push_back(corners[4]);
        marker.points.push_back(corners[7]);
        marker.points.push_back(corners[6]);
    }

};
}
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigator::obstacle_drawer::ObstacleDrawer>());
    rclcpp::shutdown();
    return 0;
}


