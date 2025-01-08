#ifndef CUGO_ROS2_CONTROL2_HPP
#define CUGO_ROS2_CONTROL2_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>

namespace cugo_ros2_control2
{

class Node : public rclcpp::Node
{
public:
  Node();
  geometry_msgs::msg::Twist last_cmd_vel;

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publishOdom();
  void checkTimeouts();
  //void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void handleSerialData(std::optional<int32_t> counter);


  // サブスクライバーとパブリッシャー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;
  //diagnostic_updater::Updater diagnostic_updater_;

  // launchファイルのパラメータ
  double control_frequency;
  //double diagnostic_frequency;
  std::string serial_port;
  int serial_baudrate;
  double cmd_vel_timeout; // /cmd_velのタイムアウト期間
  double serial_timeout;  // シリアル通信のタイムアウト期間
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_HPP_
