/*
 * Package:   zed_interface
 * Filename:  ZedInterfaceNode.hpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "std_msgs/msg/header.hpp"

#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "voltron_msgs/msg/obstacle2_d_array.hpp"

#include "cv_bridge/cv_bridge.h"

#include "sl/Camera.hpp"

namespace navigator {
namespace  zed_interface {

class ZedInterfaceNode : public rclcpp::Node {
public:
  ZedInterfaceNode();
  virtual ~ZedInterfaceNode();

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_rgb_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_pub;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle3DArray>::SharedPtr object_pub_3d;
    rclcpp::Publisher<voltron_msgs::msg::Obstacle2DArray>::SharedPtr object_pub_2d;

    sl::Camera zed;
    sl::ObjectDetectionRuntimeParameters obj_runtime_param;

    void update_camera();
    void publish_zed_img(sl::Mat mat);
    void publish_depth_img(sl::Mat mat);
    void publish_object_boxes(sl::Objects objects);
    void publish_pose(sl::Pose pose);


};
}
}
