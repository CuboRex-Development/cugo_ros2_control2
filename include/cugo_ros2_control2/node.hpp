/*
   Copyright [2025] [CuboRex Co.,Ltd.]

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef CUGO_ROS2_CONTROL2_NODE_HPP
#define CUGO_ROS2_CONTROL2_NODE_HPP

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "cugo_ros2_control2/cugo.hpp"
#include "cugo_ros2_control2/serial.hpp"

namespace cugo_ros2_control2
{

class Node : public rclcpp::Node
{
public:
  Node();

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void serial_data_callback(const std::vector<unsigned char> & body_data);
  void control_loop();
  void publish_odom_and_tf();

  // サブスクライバーとパブリッシャー
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // インスタンス
  std::unique_ptr<cugo_ros2_control2::CuGo> cugo_;
  std::shared_ptr<cugo_ros2_control2::Serial> serial_;

  // データ共有
  std::mutex data_mutex_;
  geometry_msgs::msg::Twist latest_cmd_vel_;
  rclcpp::Time last_cmd_vel_time_;
  rclcpp::Time last_serial_receive_time_;
  rclcpp::Time prev_control_loop_time_;
  //int32_t latest_left_encoder_{0};
  //int32_t latest_right_encoder_{0};
  int32_t prev_left_encoder_{0};
  int32_t prev_right_encoder_{0};
  bool is_first_serial_data_{true}; // 最初の受信データかどうかのフラグ
  double left_wheel_angle_{0.0};
  double right_wheel_angle_{0.0};

  // タイマーコールバック
  rclcpp::TimerBase::SharedPtr control_timer;
//  rclcpp::TimerBase::SharedPtr check_timeout_timer;

  // launchファイルのパラメータ
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string subscribe_topic_name;
  std::string publish_topic_name;
  double control_frequency;
  std::string serial_port;
  int serial_baudrate;
  double cmd_vel_timeout_; // /cmd_velのタイムアウト期間
  double serial_timeout_;  // シリアル通信のタイムアウト期間
  double tread;
  double l_wheel_radius, r_wheel_radius;
  double reduction_ratio;
  int encoder_resolution;
  int product_id;
  int robot_id;
  double pose_cov_x_;
  double pose_cov_y_;
  double pose_cov_z_;
  double pose_cov_roll_;
  double pose_cov_pitch_;
  double pose_cov_yaw_;
  double twist_cov_x_;
  double twist_cov_yaw_;

  std::array<double, 36> pose_covariance_{};
  std::array<double, 36> twist_covariance_{};

  // ROSでの共有データ
  double linear_x, angular_z;
  rclcpp::Time prev_recvtime_cmdvel = this->get_clock()->now();
  rclcpp::Time recvtime_cmdvel = prev_recvtime_cmdvel;
  rclcpp::Time prev_recvtime_serial = this->get_clock()->now();
  rclcpp::Time recvtime_serial = prev_recvtime_serial;
  nav_msgs::msg::Odometry current_odom_;
  sensor_msgs::msg::JointState joint_state_;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_NODE_HPP
