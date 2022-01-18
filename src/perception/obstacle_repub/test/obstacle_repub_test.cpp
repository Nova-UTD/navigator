/*
 * Package:   obstacle_repub
 * Filename:  obstacle_repub_test.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */

#include "gtest/gtest.h"
#include "zed_interfaces/msg/object.hpp"
#include "voltron_msgs/msg/obstacle3_d.hpp"


class MsgTest : public ::testing::Test {

protected:

    
    zed_interfaces::msg::Object zed_obstacle;
    voltron_msgs::msg::Obstacle3D nova_obstacle;

    void SetUp() override {
        // zed_obstacle example, a really big, demorphed person
        zed_obstacle.label = "Person";
        zed_obstacle.confidence = 99;   // for zed, b/w 1-99
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
};


// geometry_msgs require double instead floats, requires static casting to be tested
TEST_F(MsgTest, zedToNova){
    
    nova_obstacle.confidence = zed_obstacle.confidence / 100.0f;
    EXPECT_EQ(nova_obstacle.confidence, (99.0f / 100.0f));

    nova_obstacle.velocity.x = static_cast<double>(zed_obstacle.velocity.at(0));
    EXPECT_EQ(nova_obstacle.velocity.x, static_cast<double>(0.2f)); //0.2 in float casted doesn't directly equal to 0.2 in double

    nova_obstacle.velocity.y = static_cast<double>(zed_obstacle.velocity.at(1));
    EXPECT_EQ(nova_obstacle.velocity.y, -1.5);

    nova_obstacle.velocity.z = static_cast<double>(zed_obstacle.velocity.at(2));
    EXPECT_EQ(nova_obstacle.velocity.z, 0.0);

    nova_obstacle.bounding_box.size.x = static_cast<double>(zed_obstacle.dimensions_3d.at(0));
    EXPECT_EQ(nova_obstacle.bounding_box.size.x, 1.0);

    nova_obstacle.bounding_box.size.y = static_cast<double>(zed_obstacle.dimensions_3d.at(1));
    EXPECT_EQ(nova_obstacle.bounding_box.size.y, 2.0);

    nova_obstacle.bounding_box.size.z = static_cast<double>(zed_obstacle.dimensions_3d.at(2));
    EXPECT_EQ(nova_obstacle.bounding_box.size.z, 3.0);

    nova_obstacle.bounding_box.position.x = static_cast<double>(zed_obstacle.position.at(0));
    EXPECT_EQ(nova_obstacle.bounding_box.position.x, 1.5);

    nova_obstacle.bounding_box.position.y = static_cast<double>(zed_obstacle.position.at(1));
    EXPECT_EQ(nova_obstacle.bounding_box.position.y, 0.0);

    nova_obstacle.bounding_box.position.z = static_cast<double>(zed_obstacle.position.at(2));
    EXPECT_EQ(nova_obstacle.bounding_box.position.z, 1.5);

}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}
