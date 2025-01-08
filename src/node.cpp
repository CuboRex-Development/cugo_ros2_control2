#include "cugo_ros2_control2/node.hpp"

using namespace cugo_ros2_control2;
Node::Node()
: rclcpp::Node("cugo_ros2_control2")
{
  RCLCPP_INFO(this->get_logger(), "cugo_ros2_control2 has started.");

  // launchファイルからパラメータを取得
  this->declare_parameter("control_frequency", 10.0);
  this->declare_parameter("diagnostic_frequency", 1.0);
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("serial_baudrate", 115200);
  this->declare_parameter("cmd_vel_timeout", 0.5); // 秒
  this->declare_parameter("serial_timeout", 0.5);  // 秒

  this->get_parameter("control_frequency", control_frequency);
  this->get_parameter("diagnostic_frequency", diagnostic_frequency);
  this->get_parameter("serial_port", serial_port);
  this->get_parameter("serial_baudrate", serial_baudrate);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout);
  this->get_parameter("serial_timeout", serial_timeout);

  RCLCPP_INFO(this->get_logger(), "設定パラメータ");
  RCLCPP_INFO(this->get_logger(), "control_frequency: %f", control_frequency);
  RCLCPP_INFO(this->get_logger(), "diagnostic_frequency: %f", diagnostic_frequency);
  RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
  RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d", serial_baudrate);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmd_vel_timeout);
  RCLCPP_INFO(this->get_logger(), "serial_timeout: %f", serial_timeout);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cugo_ros2_control2::Node>();
  rclcpp::executors::MultiThreadedExecutor executor; // Note:シリアル通信の受信でマルチスレッド
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Cugo ROS 2 Control Node has started.");
  executor.spin();
  return 0;
}
