/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_drawer_test.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "obstacle_drawer/obstacle_drawer_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "gtest/gtest.h"
#include "voltron_test_utils/TestPublisher.hpp"
#include "voltron_test_utils/TestSubscriber.hpp"
#include "voltron_msgs/msg/obstacle3_d_array.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"
#include "obstacle_classes/obstacle_classes.hpp"
#include <memory>

using namespace Voltron::TestUtils;

class ObstacleDrawerTest : public ::testing::Test {

protected:

    std::unique_ptr<TestPublisher<voltron_msgs::msg::Obstacle3DArray>> nova_obstacle_publisher;
    std::unique_ptr<TestSubscriber<visualization_msgs::msg::MarkerArray>> marker_array_subscriber;
    std::shared_ptr<navigator::obstacle_drawer::ObstacleDrawer> obstacle_drawer_node;
    voltron_msgs::msg::Obstacle3D nova_obstacle;

    void SetUp() override {
        rclcpp::init(0, nullptr);
        nova_obstacle_publisher = std::make_unique<TestPublisher<voltron_msgs::msg::Obstacle3DArray>>("nova_obstacle_array");
        marker_array_subscriber = std::make_unique<TestSubscriber<visualization_msgs::msg::MarkerArray>>("obstacle_marker_array");
        obstacle_drawer_node = std::make_shared<navigator::obstacle_drawer::ObstacleDrawer>();

        // create an example cube obstacle going into the obstacle_drawer_node
        nova_obstacle.label = navigator::obstacle_classes::PEDESTRIAN;
        
        // TO-DO: represent nova_obstacle as a cube
    }
    void TearDown() override {
        rclcpp::shutdown();
    }

};


TEST_F(ObstacleDrawerTest, drawCube){

    // an array of one nova obstacle
    auto nova_obstacle_array_msg = voltron_msgs::msg::Obstacle3DArray();
    nova_obstacle_array_msg.obstacles.push_back(nova_obstacle);

    // send nova obstacle array msg
    nova_obstacle_publisher->send_message(nova_obstacle_array_msg);
    rclcpp::spin_some(obstacle_drawer_node);

    // check if marker array has been published by obstacle_drawer_node
    ASSERT_TRUE(marker_array_subscriber->has_message_ready());

    // receive the marker array 
    auto marker_array_received = marker_array_subscriber->get_message();

    // test if marker color is correct for a PEDESTRIAN cube

    ASSERT_EQ(marker_array_received->markers[0].color.r, 1.0);
    ASSERT_EQ(marker_array_received->markers[0].color.g, 1.0);
    ASSERT_EQ(marker_array_received->markers[0].color.b, 0.0);

    // TO-DO: check if marker coordinates are correct for a cube

}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
