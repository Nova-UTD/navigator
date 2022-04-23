/*
 * Package:   zed_interface
 * Filename:  ZedInterfaceNode.cpp
 * Author:    Jim Moore
 * Email:     jim3moore@gmail.com
 * Copyright: 2022, Voltron UTD
 * License:   MIT License
 */

#include "zed_interface/ZedInterfaceNode.hpp"

#include "cv_bridge/cv_bridge.h"

#include <chrono>


using namespace navigator::zed_interface;
using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace voltron_msgs::msg;
using namespace std::chrono_literals;

ZedInterfaceNode::ZedInterfaceNode() : rclcpp::Node("zed_interface") {
    this->pose_pub = this->create_publisher<PoseWithCovarianceStamped>("/sensors/zed/pose", 10);
    this->left_rgb_pub = this->create_publisher<Image>("sensors/zed/left_rgb", 10);
    this->depth_img_pub = this->create_publisher<Image>("sensors/zed/depth_img", 10);
    this->object_pub_3d = this->create_publisher<Obstacle3DArray>("sensors/zed/obstacle_array_3d", 10);
    this->object_pub_2d = this->create_publisher<Obstacle2DArray>("sensors/zed/obstacle_array_2d", 10);

    sl::InitParameters init_parameters;
    init_parameters.input.setFromCameraID(0);

    //Use the ROS coordinate system for all measurements
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP_X_FWD;
    init_parameters.coordinate_units = sl::UNIT::METER;  //# Set units in meters
    init_parameters.depth_minimum_distance = 1.0;
    init_parameters.depth_minimum_distance = 40.0;
    init_parameters.svo_real_time_mode = true;

    auto returned_state = zed.open(init_parameters);
    RCLCPP_INFO(this->get_logger(), "camera opened");
    if (returned_state != sl::ERROR_CODE::SUCCESS) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform base_link to map: %s", sl::toString(returned_state));
    }

    sl::PositionalTrackingParameters tracking_params;
    zed.enablePositionalTracking(tracking_params);
    RCLCPP_INFO(this->get_logger(), "tracking enabled");

    sl::BatchParameters batch_parameters;
    batch_parameters.enable = false;
    sl::ObjectDetectionParameters obj_param;
    obj_param.batch_parameters =batch_parameters;
    obj_param.detection_model=sl::DETECTION_MODEL::MULTI_CLASS_BOX;
    //# Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = true;
    zed.enableObjectDetection(obj_param);
    RCLCPP_INFO(this->get_logger(), "object detection enabled");
    
    double detection_confidence = 60;
    obj_runtime_param.detection_confidence_threshold = detection_confidence;
    obj_runtime_param.object_class_filter = {
        sl::OBJECT_CLASS::VEHICLE, sl::OBJECT_CLASS::PERSON, sl::OBJECT_CLASS::ANIMAL};
    
    obj_runtime_param.object_class_detection_confidence_threshold[sl::OBJECT_CLASS::PERSON] = detection_confidence;
    obj_runtime_param.object_class_detection_confidence_threshold[sl::OBJECT_CLASS::ANIMAL] = detection_confidence;
    obj_runtime_param.object_class_detection_confidence_threshold[sl::OBJECT_CLASS::VEHICLE] = detection_confidence;

    timer_ = this->create_wall_timer(250ms, std::bind(&ZedInterfaceNode::update_camera, this));
    RCLCPP_INFO(this->get_logger(), "wall timer");
}

void ZedInterfaceNode::update_camera() {
    RCLCPP_INFO(this->get_logger(), "start outgoing camera");
    sl::RuntimeParameters runtime;
    sl::Pose camera_pose;
    sl::Transform pose_data;
    sl::Mat image;
    sl::Mat depth;
    sl::Objects objects;

    if (zed.grab(runtime) == sl::ERROR_CODE::SUCCESS) {
        zed.retrieveImage(image, sl::VIEW::LEFT);
        zed.retrieveMeasure(depth, sl::MEASURE::DEPTH);
        zed.retrieveObjects(objects, obj_runtime_param);
        auto tracking_state = zed.getPosition(camera_pose, sl::REFERENCE_FRAME::WORLD);
        if (tracking_state == sl::POSITIONAL_TRACKING_STATE::OK) {
            publish_pose(camera_pose);
        } else {
            RCLCPP_WARN(this->get_logger(), "Positional tracking not available");
        }
        publish_zed_img(image);
        publish_depth_img(depth);
        publish_object_boxes(objects);
        RCLCPP_INFO(this->get_logger(), "publish");
    }

}

//from https://github.com/stereolabs/zed-opencv/blob/master/cpp/src/main.cpp
// Mapping between MAT_TYPE and CV_TYPE
int getOCVtype(sl::MAT_TYPE type) {
    int cv_type = -1;
    switch (type) {
        case sl::MAT_TYPE::F32_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE::F32_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE::F32_C3: cv_type = CV_32FC3; break;
        case sl::MAT_TYPE::F32_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE::U8_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE::U8_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE::U8_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE::U8_C4: cv_type = CV_8UC4; break;
        default: break;
    }
    return cv_type;
}

//from https://github.com/stereolabs/zed-opencv/blob/master/cpp/src/main.cpp
cv::Mat slMat2cvMat(sl::Mat& input) {
    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), getOCVtype(input.getDataType()), input.getPtr<sl::uchar1>(sl::MEM::CPU), input.getStepBytes(sl::MEM::CPU));
}

void ZedInterfaceNode::publish_zed_img(sl::Mat mat) {
    cv::Mat img = slMat2cvMat(mat);
    
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image msg; // >> message to be sent

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(msg); // from cv_bridge to sensor_msgs::Image
    msg.header.frame_id = "zed2_camera_center";
    left_rgb_pub->publish(msg);
}

void ZedInterfaceNode::publish_depth_img(sl::Mat mat) {
    cv::Mat img = slMat2cvMat(mat);
    
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image msg; // >> message to be sent

    std_msgs::msg::Header header; // empty header
    header.stamp = get_clock()->now(); // time
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
    img_bridge.toImageMsg(msg); // from cv_bridge to sensor_msgs::Image
    msg.header.frame_id = "zed2_camera_center";
    depth_img_pub->publish(msg);
}

void ZedInterfaceNode::publish_object_boxes(sl::Objects objects) {
    Obstacle3DArray obj_array;
    Obstacle2DArray obj_2d_array;
    for(auto object : objects.object_list){
        Obstacle3D obj_msg;
        Obstacle2D obj_2d_msg;
        if (object.label == sl::OBJECT_CLASS::PERSON) {
                obj_msg.label = Obstacle3D::PEDESTRIAN;
                obj_2d_msg.label = Obstacle3D::PEDESTRIAN;
        }
        else if (object.label == sl::OBJECT_CLASS::VEHICLE) {
            if (object.sublabel == sl::OBJECT_SUBCLASS::BICYCLE) {
                obj_msg.label = Obstacle3D::BIKE;
                obj_2d_msg.label = Obstacle3D::BIKE;
            }
            else {
                obj_msg.label = Obstacle3D::CAR;
                obj_2d_msg.label = Obstacle3D::CAR;
            }
        }
        else {
            obj_msg.label = Obstacle3D::OTHER;
            obj_2d_msg.label = Obstacle3D::OTHER;
        }
        obj_msg.id = object.id;
        obj_2d_msg.id = object.id;
        obj_msg.confidence = object.confidence;
        obj_2d_msg.confidence = object.confidence;

        obj_msg.velocity.x = object.velocity[0];
        obj_msg.velocity.y = object.velocity[1];
        obj_msg.velocity.z = object.velocity[2];

        if (object.bounding_box.size() == 8) {
            //have enough corner for bounding box
            for (size_t i = 0; i < 8; i++) {
                geometry_msgs::msg::Point pt;
                pt.x = object.bounding_box[i][0];
                pt.y = object.bounding_box[i][1];
                pt.z = object.bounding_box[i][2];
                obj_msg.bounding_box.corners[i] = pt;
            }
        }
        obj_array.obstacles.push_back(obj_msg);
        if (object.bounding_box_2d.size() == 4) {
            //enough for corners of bbox 2d
            obj_2d_msg.bounding_box.a[0] = object.bounding_box_2d[0][0];
            obj_2d_msg.bounding_box.a[1] = object.bounding_box_2d[0][1];
            obj_2d_msg.bounding_box.b[0] = object.bounding_box_2d[1][0];
            obj_2d_msg.bounding_box.b[1] = object.bounding_box_2d[1][1];
            obj_2d_msg.bounding_box.c[0] = object.bounding_box_2d[2][0];
            obj_2d_msg.bounding_box.c[1] = object.bounding_box_2d[2][1];
            obj_2d_msg.bounding_box.d[0] = object.bounding_box_2d[3][0];
            obj_2d_msg.bounding_box.d[1] = object.bounding_box_2d[3][1];   
        }
        obj_2d_array.obstacles.push_back(obj_2d_msg);

        obj_array.header.stamp = get_clock()->now();
        obj_array.header.frame_id = "base_link";
        obj_2d_array.header.stamp = get_clock()->now();
        obj_2d_array.header.frame_id = "base_link";

        object_pub_3d->publish(obj_array);
        object_pub_2d->publish(obj_2d_array);
    }

}

void ZedInterfaceNode::publish_pose(sl::Pose pose) {
    //auto rotation = pose.getRotationVector();
    auto translation = pose.getTranslation();

    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = get_clock()->now();
    //# Hm...  We will set our state estimate UKF to differentiate this.
    msg.header.frame_id = "map";
    //# Set position
    msg.pose.pose.position.x = translation[0];
    msg.pose.pose.position.y = translation[1];
    //# This should be zero... but it isn't
    msg.pose.pose.position.z = translation[2];
    //# Set orientation
    //auto quat = R.from_rotvec(rotation).as_quat();  //# [x,y,z,w]
    auto quat = pose.getOrientation();
    msg.pose.pose.orientation.x = quat[0];
    msg.pose.pose.orientation.y = quat[1];
    msg.pose.pose.orientation.z = quat[2];
    msg.pose.pose.orientation.w = quat[3];
    pose_pub->publish(msg);
}

ZedInterfaceNode::~ZedInterfaceNode() {}