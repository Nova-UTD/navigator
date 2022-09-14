/*
 * Package:   obstacle_repub
 * Filename:  obstacle_repub_test.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "zed_interfaces/msg/object.hpp"
#include "zed_interfaces/msg/objects_stamped.h"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "voltron_test_utils/TestPublisher.hpp"
#include "voltron_test_utils/TestSubscriber.hpp"
#include "obstacle_repub/obstacle_repub_node.hpp"
#include "obstacle_classes/obstacle_classes.hpp"
#include <memory>

using namespace Voltron::TestUtils;

class ObstacleRepubTest : public ::testing::Test {

protected:
    
    zed_interfaces::msg::Object zed_obstacle;
    std::unique_ptr<TestPublisher<zed_interfaces::msg::ObjectsStamped>> zed_obstacle_publisher;
    std::unique_ptr<TestSubscriber<voltron_msgs::msg::Obstacle3DArray>> nova_obstacle_subscriber;
    std::shared_ptr<navigator::obstacle_repub::ObstacleRepublisher> obstacle_republisher;

    void SetUp() override {
        rclcpp::init(0, nullptr);
        zed_obstacle_publisher = std::make_unique<TestPublisher<zed_interfaces::msg::ObjectsStamped>>("zed_obstacle_array");
        nova_obstacle_subscriber = std::make_unique<TestSubscriber<voltron_msgs::msg::Obstacle3DArray>>("nova_obstacle_array");
        obstacle_republisher = std::make_shared<navigator::obstacle_repub::ObstacleRepublisher>();

        // zed obstacle example
        zed_obstacle.label = "Person";
        zed_obstacle.confidence = 86;   // for zed should be b/w 1-99
        zed_obstacle.position = {1.5f, 0.0f, 1.5f}; 
        zed_obstacle.velocity = {0.2f, -1.5f, 0.0f};  
        zed_obstacle.dimensions_3d = {1.0f, 2.0f, 3.0f};
        zed_obstacle.bounding_box_3d.corners[0].kp = {1.0f, 1.0f, 3.0f};
        zed_obstacle.bounding_box_3d.corners[1].kp = {2.0f, 1.0f, 3.0f};
        zed_obstacle.bounding_box_3d.corners[2].kp = {2.0f, -1.0f, 3.0f};
        zed_obstacle.bounding_box_3d.corners[3].kp = {1.0f, -1.0f, 3.0f};
        zed_obstacle.bounding_box_3d.corners[4].kp = {1.0f, 1.0f, 0.0f};
        zed_obstacle.bounding_box_3d.corners[5].kp = {2.0f, 1.0f, 0.0f};
        zed_obstacle.bounding_box_3d.corners[6].kp = {2.0f, -1.0f, 0.0f};
        zed_obstacle.bounding_box_3d.corners[7].kp = {1.0f, -1.0f, 0.0f};

    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};


TEST_F(ObstacleRepubTest, zedMsgRepub){

    // an array of one zed obstacle
    auto zed_obstacle_array_msg = zed_interfaces::msg::ObjectsStamped();
    zed_obstacle_array_msg.objects.push_back(zed_obstacle);

    // send zed msg
    zed_obstacle_publisher->send_message(zed_obstacle_array_msg);
    rclcpp::spin_some(obstacle_republisher);

    ASSERT_TRUE(nova_obstacle_subscriber->has_message_ready());

    // receive the republished msg in standard format for this project
    auto msg_received = nova_obstacle_subscriber->get_message();
    auto nova_obstacle = msg_received->obstacles[0];

    // check if "Person" label on zed msg gets encoded to PEDESTRIAN enum definition
    ASSERT_EQ(nova_obstacle.label, navigator::obstacle_classes::PEDESTRIAN);

    // check if confidence got translated between 0.0-0.99
    ASSERT_EQ(nova_obstacle.confidence, 0.86f);

    // some value checks
    ASSERT_EQ(nova_obstacle.bounding_box.center.position.x, 1.5);
    ASSERT_EQ(nova_obstacle.velocity.y, -1.5);
    ASSERT_EQ(nova_obstacle.bounding_box.size.z, 3.0);
    ASSERT_EQ(nova_obstacle.bounding_box.corners[3].y, -1.0); 

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
