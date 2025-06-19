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

#include "cugo_ros2_control2/node.hpp"

using namespace cugo_ros2_control2;
Node::Node()
: rclcpp::Node("cugo_ros2_control2")
{
  RCLCPP_INFO(this->get_logger(), "cugo_ros2_control2 has started.");

  // launchファイルからパラメータを取得
  this->declare_parameter("odom_frame_id", "odom");
  this->declare_parameter("base_link_frame_id", "base_link");
  this->declare_parameter("subscribe_topic_name", "/cmd_vel");
  this->declare_parameter("publish_topic_name", "/odom");
  this->declare_parameter("control_frequency", 10.0);
  //this->declare_parameter("diagnostic_frequency", 1.0);
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("serial_baudrate", 115200);
  this->declare_parameter("cmd_vel_timeout", 0.5); // 秒
  this->declare_parameter("serial_timeout", 0.5);  // 秒
  this->declare_parameter("tread", 0.376);
  this->declare_parameter("l_wheel_radius", 0.03858);
  this->declare_parameter("r_wheel_radius", 0.03858);
  this->declare_parameter("reduction_ratio", 20.0);
  this->declare_parameter("encoder_resolution", 30);

  this->get_parameter("odom_frame_id", odom_frame_id);
  this->get_parameter("base_link_frame_id", base_link_frame_id);
  this->get_parameter("subscribe_topic_name", subscribe_topic_name);
  this->get_parameter("publish_topic_name", publish_topic_name);
  this->get_parameter("control_frequency", control_frequency);
  //this->get_parameter("diagnostic_frequency", diagnostic_frequency);
  this->get_parameter("serial_port", serial_port);
  this->get_parameter("serial_baudrate", serial_baudrate);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout);
  this->get_parameter("serial_timeout", serial_timeout);
  this->get_parameter("tread", tread);
  this->get_parameter("l_wheel_radius", l_wheel_radius);
  this->get_parameter("r_wheel_radius", r_wheel_radius);
  this->get_parameter("reduction_ratio", reduction_ratio);
  this->get_parameter("encoder_resolution", encoder_resolution);

  RCLCPP_INFO(this->get_logger(), "設定パラメータ");
  RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "base_link_frame_id: %s", base_link_frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "subscribe_topic_name: %s", subscribe_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_topic_name: %s", publish_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "control_frequency: %f", control_frequency);
  //RCLCPP_INFO(this->get_logger(), "diagnostic_frequency: %f", diagnostic_frequency);
  RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
  RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d", serial_baudrate);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmd_vel_timeout);
  RCLCPP_INFO(this->get_logger(), "serial_timeout: %f", serial_timeout);
  RCLCPP_INFO(this->get_logger(), "tread: %f", tread);
  RCLCPP_INFO(this->get_logger(), "l_wheel_radius: %f", l_wheel_radius);
  RCLCPP_INFO(this->get_logger(), "r_wheel_radius: %f", r_wheel_radius);
  RCLCPP_INFO(this->get_logger(), "reduction_ratio: %f", reduction_ratio);
  RCLCPP_INFO(this->get_logger(), "encoder_resolution: %d", encoder_resolution);

  // 各クラスの初期化
  cugo_ = std::make_unique<cugo_ros2_control2::CuGo>(
      l_wheel_radius, r_wheel_radius, tread, reduction_ratio, encoder_resolution
  );
  serial_ = std::make_shared<cugo_ros2_control2::Serial>();

  // Serial通信の開始
  try {
    serial_->open(serial_port, serial_baudrate);
    serial_->register_callback(
        std::bind(&Node::serial_data_callback, this, std::placeholders::_1)
    );
    serial_->start_read(); // 受信ループを開始
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Failed to setup serial communication: %s. Shutting down.",
      e.what());
    rclcpp::shutdown();
    return;
  }

  // 状態変数の初期化
  auto now = this->get_clock()->now();
  last_cmd_vel_time_ = now;
  last_serial_receive_time_ = now;
  prev_control_loop_time_ = now;
  current_odom_.header.frame_id = odom_frame_id;
  current_odom_.child_frame_id = base_link_frame_id;

  // ROS トピック通信
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    subscribe_topic_name.c_str(), 1,
    std::bind(&Node::cmd_vel_callback, this, std::placeholders::_1));
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(publish_topic_name.c_str(), 10);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);


  // ループ処理の開始
  control_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
    std::bind(&Node::control_loop, this)
  );

  // 失敗フラグがあれば1秒おきに通知
  /*
  check_timeout_timer = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&Node::notify_message, this)
  );
  */
}

void Node::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  // 先にタイムスタンプを取得
  rclcpp::Time reception_time = this->get_clock()->now();

  // lock_guardでmutexをロックし、スコープを抜けたら自動でアンロック
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 共有データを更新
  latest_cmd_vel_ = *msg;
  last_cmd_vel_time_ = reception_time;
}

// シリアルデータ受信時のコールバック (Serialクラスから呼ばれる)
void Node::serial_data_callback(const std::vector<unsigned char> & body_data)
{
  // 先にタイムスタンプを取得
  rclcpp::Time reception_time = this->get_clock()->now();

  // lock_guardでmutexをロック
  std::lock_guard<std::mutex> lock(data_mutex_);

  // 共有データを更新
  // ボディデータからエンコーダ値を読み出す (ボディの0バイト目から左、4バイト目から右)
  latest_left_encoder_ = cugo_ros2_control2::Serial::bin_to_int32(body_data.data() + 0);
  latest_right_encoder_ = cugo_ros2_control2::Serial::bin_to_int32(body_data.data() + 4);
  last_serial_receive_time_ = reception_time;

  // 最初のデータを受信したら、prev_encoder も初期化する
  if (is_first_serial_data_) {
    prev_left_encoder_ = latest_left_encoder_;
    prev_right_encoder_ = latest_right_encoder_;
    is_first_serial_data_ = false;
  }
}

bool Node::is_timeout(double current_time, double prev_time, double timeout_duration)
{
  return check_difftime(current_time, prev_time) >= timeout_duration;
}

bool Node::is_sametime(double current_time, double prev_time)
{
  double dt = check_difftime(current_time, prev_time);
  // 更新がないかチェック。なければ同じタイムスタンプを見るため完全一致
  return std::abs(dt) < 1e-6;
}

bool Node::is_illegaltime(double current_time, double prev_time)
{
  return check_difftime(current_time, prev_time) < 0.0;
}

double Node::check_difftime(double current_time, double prev_time)
{
  return current_time - prev_time;
}

RPM Node::set_zero_rpm()
{
  RPM rpm;
  rpm.l_rpm = 0.0f;
  rpm.r_rpm = 0.0f;
  return rpm;
}

void Node::control_loop()
{
  RCLCPP_DEBUG(this->get_logger(), "10Hz Job");
}

void Node::notify_message()
{
  RCLCPP_DEBUG(this->get_logger(), "1Hz Job");
}
