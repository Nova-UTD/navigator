#include <memory>

#include "localization/LocalizationNode.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto localization_node =
      std::make_shared<navigator::localization::LocalizationNode>();
  executor.add_node(localization_node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
