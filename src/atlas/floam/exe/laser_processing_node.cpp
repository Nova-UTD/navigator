#include "floam/LaserProcessingNode.hpp"

using namespace Nova::Atlas;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LaserProcessingNode>());

    rclcpp::shutdown();

    return 0;
}