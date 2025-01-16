#include "cugo_ros2_control2/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cugo_ros2_control2::Node>();
  rclcpp::executors::MultiThreadedExecutor executor; // マルチスレッドエグゼキュータ
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Cugo ROS 2 Control Node has started.");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
