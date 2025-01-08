#include "cugo_ros2_control2/node.hpp"

using namespace cugo_ros2_control2;
Node::Node()
: rclcpp::Node("cugo_ros2_control2")
{
  RCLCPP_INFO(this->get_logger(), "Cugo ROS 2 Control Node has started.");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("cugo_ros2_control2");
  RCLCPP_INFO(node->get_logger(), "Cugo ROS 2 Control Node has started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
