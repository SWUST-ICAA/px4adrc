#include <rclcpp/rclcpp.hpp>

#include "px4adrc/adrc_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<px4adrc::AdrcNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
