/*
 * Package:   obstacle_drawer
 * Filename:  obstacle_drawer.cpp
 * Author:    Ragib "Rae" Arnab
 * Email:     ria190000@utdallas.edu
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 */


#include "obstacle_drawer/obstacle_drawer_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<navigator::obstacle_drawer::ObstacleDrawer>());
    rclcpp::shutdown();
    return 0;
}
