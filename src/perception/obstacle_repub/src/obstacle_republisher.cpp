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
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"

#include <memory>

using std::placeholders::_1;

class ObstacleRepublisher : public rclcpp::Node {

public:
    ObstacleRepublisher() : Node("obstacle_republisher"){
        // creates a subscription that listens to the topic published from SVL's 3D Ground Truth sensor
        // and then calls back to a private method in this class definition
        svl_obstacle_subscription = this->create_subscription<lgsvl_msgs::msg::Detection3DArray>(
        "/ground_truth_3d/detections", 10, std::bind(&ObstacleRepublisher::svl_obstacle_callback, this, _1));

        // an publishers that republishes 3D vehicle detections into our own abstract format
        std_vehicle_publisher = this->create_publisher<voltron_msgs::msg::Obstacle3DArray>("/obstacles/vehicles", 10);

        // an publishers that republishes 3D pedestrian detections into our own abstract format
        std_pedestrian_publisher = this->create_publisher<voltron_msgs::msg::Obstacle3DArray>("/obstacles/pedestrians", 10);
    }
private:

    rclcpp::Subscription<lgsvl_msgs::msg::Detection3DArray>::SharedPtr svl_obstacle_subscription;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr std_vehicle_publisher;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr std_pedestrian_publisher;

    void svl_obstacle_callback(const lgsvl_msgs::msg::Detection3DArray::SharedPtr msg) const {

        auto std_vehicle_array = voltron_msgs::msg::Obstacle3DArray();
        auto std_pedestrian_array = voltron_msgs::msg::Obstacle3DArray();

        for (const auto detection : msg->detections) {
            auto std_obstacle = voltron_msgs::msg::Obstacle3D();

            std_obstacle.id = detection.id;
            std_obstacle.confidence = detection.score;
            std_obstacle.velocity = detection.velocity.linear;
            std_obstacle.bounding_box.box_pose = detection.bbox.position;
            std_obstacle.bounding_box.box_size = detection.bbox.size;

            if (detection.label != "Pedestrian"){
                std_obstacle.label = "Vehicle";
                std_vehicle_array.obstacles.push_back(std_obstacle);
            }
            else{
                std_obstacle.label = detection.label;
                std_pedestrian_array.obstacles.push_back(std_obstacle);
            }
        }

        std_vehicle_publisher->publish(std_vehicle_array);
        std_pedestrian_publisher->publish(std_pedestrian_array);
    }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleRepublisher>());
  rclcpp::shutdown();
  return 0;
}



