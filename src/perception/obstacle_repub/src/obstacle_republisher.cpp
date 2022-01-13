/*
 * Package:   obstacle_repub
 * Filename:  obstacle_republisher.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "rclcpp/rclcpp.hpp"
#include "lgsvl_msgs/msg/detection3_d_array.hpp"
#include "zed_interfaces/msg/objects_stamped.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "../include/obstacle_repub/obstacle_classes.hpp"
#include <memory>

using std::placeholders::_1;

namespace navigator {
namespace obstacle_repub {

class ObstacleRepublisher : public rclcpp::Node {

public:
    ObstacleRepublisher() : Node("obstacle_republisher"){
        // creates a subscription that listens to the object detection topic published from SVL's 3D Ground Truth sensor
        // and then calls back to a private method in this class definition
        svl_obstacle_subscription = this->create_subscription<lgsvl_msgs::msg::Detection3DArray>(
        "/ground_truth_3d/detections", 10, std::bind(&ObstacleRepublisher::svl_obstacle_callback, this, _1));

        // creates a subscription that listens to the object detection topic published from ZED Ros2 Node 
        // and then calls back to a private method in this class definition
        zed_obstacle_subscription = this->create_subscription<zed_interfaces::msg::ObjectsStamped>(
        "/obj_det/objects", 10, std::bind(&ObstacleRepublisher::zed_obstacle_callback, this, _1));

        // an publishers that republishes 3D vehicle detections into our own abstract format
        std_vehicle_publisher = this->create_publisher<voltron_msgs::msg::Obstacle3DArray>("/obstacles/vehicles", 10);

        // an publishers that republishes 3D pedestrian detections into our own abstract format
        std_pedestrian_publisher = this->create_publisher<voltron_msgs::msg::Obstacle3DArray>("/obstacles/pedestrians", 10);
    }

private:

    rclcpp::Subscription<lgsvl_msgs::msg::Detection3DArray>::SharedPtr svl_obstacle_subscription;
    rclcpp::Subscription<zed_interfaces::msg::ObjectsStamped>::SharedPtr zed_obstacle_subscription;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr std_vehicle_publisher;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr std_pedestrian_publisher;

    void svl_obstacle_callback(const lgsvl_msgs::msg::Detection3DArray::SharedPtr msg) const {

        auto std_vehicle_array = voltron_msgs::msg::Obstacle3DArray();
        auto std_pedestrian_array = voltron_msgs::msg::Obstacle3DArray();
        std_vehicle_array.header = msg->header;
        std_pedestrian_array.header = msg->header;

        for (const auto detection : msg->detections) {
            auto std_obstacle = voltron_msgs::msg::Obstacle3D();

            std_obstacle.id = detection.id;
            std_obstacle.confidence = detection.score;
            std_obstacle.velocity = detection.velocity.linear;
            std_obstacle.bounding_box.size = detection.bbox.size;
  
            // maping each index of corners array into this format
            //      1 ------- 2
            //     /.        /|
            //    0 ------- 3 |               +z        x = forward
            //    | .       | |               |         y = left
            //    | 5.......| 6               | +x      z = up
            //    |.        |/                |/
            //    4 ------- 7       +y -------*

            auto box_transform = geometry_msgs::msg::TransformStamped();
            box_transform.transform.set__rotation(detection.bbox.position.orientation);
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
                tranformed.point = std_obstacle.bounding_box.corners[i];

                tf2::doTransform(to_tranform, tranformed, box_transform);
    
                std_obstacle.bounding_box.corners[i].x = tranformed.point.x + detection.bbox.position.position.x;
                std_obstacle.bounding_box.corners[i].y = tranformed.point.y + detection.bbox.position.position.y;
                std_obstacle.bounding_box.corners[i].z = tranformed.point.z + detection.bbox.position.position.z;
            }   

            std_obstacle.bounding_box.position = detection.bbox.position.position;

            if (detection.label != "Pedestrian"){
                std_obstacle.label = navigator::obstacle_repub::OBSTACLE_CLASS::VEHICLE;
                std_vehicle_array.obstacles.push_back(std_obstacle);
            }
            else{
                std_obstacle.label = navigator::obstacle_repub::OBSTACLE_CLASS::PEDESTRIAN;
                std_pedestrian_array.obstacles.push_back(std_obstacle);
            }
        }

        std_vehicle_publisher->publish(std_vehicle_array);
        std_pedestrian_publisher->publish(std_pedestrian_array);
    }

    void zed_obstacle_callback(const zed_interfaces::msg::ObjectsStamped::SharedPtr msg) const {
        auto std_vehicle_array = voltron_msgs::msg::Obstacle3DArray();
        auto std_pedestrian_array = voltron_msgs::msg::Obstacle3DArray();
        std_vehicle_array.header = msg->header;
        std_pedestrian_array.header = msg->header;
        
        for (const auto detection : msg->objects) {
            auto std_obstacle = voltron_msgs::msg::Obstacle3D();

            std_obstacle.id = detection.label_id;
            
            std_obstacle.confidence = detection.confidence / 100.0f;

            std_obstacle.velocity.x = static_cast<double>(detection.velocity.at(0));
            std_obstacle.velocity.y = static_cast<double>(detection.velocity.at(1));
            std_obstacle.velocity.z = static_cast<double>(detection.velocity.at(2));

            std_obstacle.bounding_box.size.x = static_cast<double>(detection.dimensions_3d.at(0));
            std_obstacle.bounding_box.size.y = static_cast<double>(detection.dimensions_3d.at(1));
            std_obstacle.bounding_box.size.z = static_cast<double>(detection.dimensions_3d.at(2));

            std_obstacle.bounding_box.position.x = static_cast<double>(detection.position.at(0));
            std_obstacle.bounding_box.position.y = static_cast<double>(detection.position.at(1));
            std_obstacle.bounding_box.position.z = static_cast<double>(detection.position.at(2));
            
            for (int i = 0; i < 8; i++){
                std_obstacle.bounding_box.corners[i].x = static_cast<double>(detection.bounding_box_3d.corners[i].kp[0]);
                std_obstacle.bounding_box.corners[i].y = static_cast<double>(detection.bounding_box_3d.corners[i].kp[1]);
                std_obstacle.bounding_box.corners[i].z = static_cast<double>(detection.bounding_box_3d.corners[i].kp[2]);
            }

            if (detection.label == "Vehicle"){
                std_obstacle.label = navigator::obstacle_repub::OBSTACLE_CLASS::VEHICLE;
                std_vehicle_array.obstacles.push_back(std_obstacle);
            }
            else if (detection.label == "Person"){
                std_obstacle.label = navigator::obstacle_repub::OBSTACLE_CLASS::PEDESTRIAN;
                std_pedestrian_array.obstacles.push_back(std_obstacle);
            }

        }
        std_vehicle_publisher->publish(std_vehicle_array);
        std_pedestrian_publisher->publish(std_pedestrian_array);
    }

};
}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<navigator::obstacle_repub::ObstacleRepublisher>());
  rclcpp::shutdown();
  return 0;
}



