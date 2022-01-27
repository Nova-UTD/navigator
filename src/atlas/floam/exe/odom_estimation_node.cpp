#include "floam/OdomEstimationNode.hpp"

using namespace Nova::Atlas;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<OdomEstimationNode>());

    rclcpp::shutdown();

    return 0;
}