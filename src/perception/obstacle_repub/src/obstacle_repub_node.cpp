/*
 * Package:   obstacle_repub
 * Filename:  obstacle_repub_node.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "obstacle_repub/obstacle_repub_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "lgsvl_msgs/msg/detection3_d_array.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "obstacle_classes/obstacle_classes.hpp"
#include <memory>

using std::placeholders::_1;

namespace navigator {
namespace obstacle_repub {

ObstacleRepublisher::ObstacleRepublisher() : Node("obstacle_republisher"){
    // creates a subscription that listens to the object detection topic published from SVL's 3D Ground Truth sensor
    // and then calls back to a private method in this class definition
    svl_obstacle_subscription = this->create_subscription<lgsvl_msgs::msg::Detection3DArray>(
    "svl_obstacle_array", 10, std::bind(&ObstacleRepublisher::svl_obstacle_callback, this, _1));

    // creates a subscription that listens to the object detection topic published from ZED Ros2 Node 
    // and then calls back to a private method in this class definition
    zed_obstacle_subscription = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
    "zed_obstacle_array", 10, std::bind(&ObstacleRepublisher::zed_obstacle_callback, this, _1));

    // a publisher that republishes 3D vehicle detections into our own abstract format
    nova_obstacle_publisher = this->create_publisher<voltron_msgs::msg::Obstacle3DArray>("nova_obstacle_array", 10);
}

void ObstacleRepublisher::svl_obstacle_callback(const lgsvl_msgs::msg::Detection3DArray::SharedPtr msg) const {

    auto nova_obstacle_array = voltron_msgs::msg::Obstacle3DArray();
    nova_obstacle_array.header.frame_id = msg->header.frame_id;
    nova_obstacle_array.header.stamp = this->now();

    for (const auto detection : msg->detections) {
        auto nova_obstacle = voltron_msgs::msg::Obstacle3D();

        nova_obstacle.id = detection.id;
        nova_obstacle.confidence = detection.score;
        nova_obstacle.velocity = detection.velocity.linear;
        nova_obstacle.bounding_box.size = detection.bbox.size;

        // mapping each index of corners array into this format
        //      1 ------- 2
        //     /.        /|
        //    0 ------- 3 |               +z        x = forward
        //    | .       | |               |         y = left
        //    | 5.......| 6               | +x      z = up
        //    |.        |/                |/
        //    4 ------- 7       +y -------*

        auto box_transform = geometry_msgs::msg::TransformStamped();
        box_transform.transform.rotation = detection.bbox.position.orientation;
        geometry_msgs::msg::Point temp_corners[8];

        for (int i = 0; i < 8; i++){
            if (i % 4 == 1 || i % 4 == 2)
                temp_corners[i].x = detection.bbox.size.x / 2.0;
            else
                temp_corners[i].x = -detection.bbox.size.x / 2.0;
            if (i % 4 == 0 || i % 4 == 1)
                temp_corners[i].y = detection.bbox.size.y / 2.0;
            else 
                temp_corners[i].y = -detection.bbox.size.y / 2.0;
            if (i < 4)
                temp_corners[i].z = detection.bbox.size.z / 2.0;
            else 
                temp_corners[i].z = -detection.bbox.size.z / 2.0;

            // for some reason, it has to be PointStamped for doTransform to work
            auto to_tranform = geometry_msgs::msg::PointStamped();
            to_tranform.point = temp_corners[i];
            auto tranformed = geometry_msgs::msg::PointStamped();
            tranformed.point = nova_obstacle.bounding_box.corners[i];

            tf2::doTransform(to_tranform, tranformed, box_transform);

            nova_obstacle.bounding_box.corners[i].x = tranformed.point.x + detection.bbox.position.position.x;
            nova_obstacle.bounding_box.corners[i].y = tranformed.point.y + detection.bbox.position.position.y;
            nova_obstacle.bounding_box.corners[i].z = tranformed.point.z + detection.bbox.position.position.z;
        }   

        nova_obstacle.bounding_box.position = detection.bbox.position.position;

        if (detection.label != "Pedestrian"){
            nova_obstacle.label = navigator::obstacle_classes::OBSTACLE_CLASS::VEHICLE;
            nova_obstacle_array.obstacles.push_back(nova_obstacle);
        }
        else {
            nova_obstacle.label = navigator::obstacle_classes::OBSTACLE_CLASS::PEDESTRIAN;
            nova_obstacle_array.obstacles.push_back(nova_obstacle);
        }
    }

    nova_obstacle_publisher->publish(nova_obstacle_array);
}

void ObstacleRepublisher::zed_obstacle_callback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) const {
    auto nova_obstacle_array = voltron_msgs::msg::Obstacle3DArray();
    nova_obstacle_array.header.frame_id = msg->header.frame_id;
    nova_obstacle_array.header.stamp = this->now();
    
    for (const auto detection : msg->objects) {
        auto nova_obstacle = voltron_msgs::msg::Obstacle3D();

        nova_obstacle.id = detection.label_id;
        
        nova_obstacle.confidence = detection.confidence / 100.0f;

        nova_obstacle.velocity.x = static_cast<double>(detection.velocity.at(0));
        nova_obstacle.velocity.y = static_cast<double>(detection.velocity.at(1));
        nova_obstacle.velocity.z = static_cast<double>(detection.velocity.at(2));

        nova_obstacle.bounding_box.size.x = static_cast<double>(detection.dimensions_3d.at(0));
        nova_obstacle.bounding_box.size.y = static_cast<double>(detection.dimensions_3d.at(1));
        nova_obstacle.bounding_box.size.z = static_cast<double>(detection.dimensions_3d.at(2));

        nova_obstacle.bounding_box.position.x = static_cast<double>(detection.position.at(0));
        nova_obstacle.bounding_box.position.y = static_cast<double>(detection.position.at(1));
        nova_obstacle.bounding_box.position.z = static_cast<double>(detection.position.at(2));
        
        for (int i = 0; i < 8; i++){
            nova_obstacle.bounding_box.corners[i].x = static_cast<double>(detection.bounding_box_3d.corners[i].kp[0]);
            nova_obstacle.bounding_box.corners[i].y = static_cast<double>(detection.bounding_box_3d.corners[i].kp[1]);
            nova_obstacle.bounding_box.corners[i].z = static_cast<double>(detection.bounding_box_3d.corners[i].kp[2]);
        }

        // this needs to be tested
        if (detection.label == "Vehicle"){
            nova_obstacle.label = navigator::obstacle_classes::OBSTACLE_CLASS::VEHICLE;
            nova_obstacle_array.obstacles.push_back(nova_obstacle);
        }
        else if (detection.label == "Person"){
            nova_obstacle.label = navigator::obstacle_classes::OBSTACLE_CLASS::PEDESTRIAN;
            nova_obstacle_array.obstacles.push_back(nova_obstacle);
        }

    }
    nova_obstacle_publisher->publish(nova_obstacle_array);
}

}
}
