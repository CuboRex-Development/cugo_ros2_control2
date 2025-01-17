#ifndef CUGO_ROS2_CONTROL2_HPP
#define CUGO_ROS2_CONTROL2_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "cugo_ros2_control2/cugo.hpp"

namespace cugo_ros2_control2
{

class Node : public rclcpp::Node
{
public:
  Node();
  //geometry_msgs::msg::Twist last_cmd_vel;
  double check_difftime(double recvtime, double prev_recvtime);
  bool is_timeout(double recvtime, double prev_recvtime, double timeout_duration);
  bool is_sametime(double recvtime, double prev_recvtime);
  bool is_illegaltime(double recvtime, double prev_recvtime);
  RPM set_zero_rpm();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void publish_odom();
  void control();
  void notify_message();
  //void updateDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void handle_serial_data(std::optional<int32_t> counter);
  void timer_loop();

  // サブスクライバーとパブリッシャー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::TimerBase::SharedPtr odom_timer;
  rclcpp::TimerBase::SharedPtr timeout_timer;
  //diagnostic_updater::Updater diagnostic_updater;

  // タイマーコールバック
  rclcpp::TimerBase::SharedPtr control_timer;
  rclcpp::TimerBase::SharedPtr check_timeout_timer;

  // launchファイルのパラメータ
  std::string subscribe_topic_name;
  std::string publish_topic_name;
  double control_frequency;
  //double diagnostic_frequency;
  std::string serial_port;
  int serial_baudrate;
  double cmd_vel_timeout; // /cmd_velのタイムアウト期間
  double serial_timeout;  // シリアル通信のタイムアウト期間
  double tread;
  double l_wheel_radius, r_wheel_radius;
  double reduction_ratio;
  int encoder_resolution;

  double linear_x, angular_z;
  rclcpp::Time prev_recvtime_cmdvel = this->get_clock()->now();
  rclcpp::Time recvtime_cmdvel = prev_recvtime_cmdvel;
  rclcpp::Time prev_recvtime_serial = this->get_clock()->now();
  rclcpp::Time recvtime_serial = prev_recvtime_serial;

  Twist last_cmd_vel;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_HPP_
