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
  this->declare_parameter("serial_port", "/dev/ttyACM0");
  this->declare_parameter("serial_baudrate", 115200);
  this->declare_parameter("cmd_vel_timeout", 0.5); // 秒
  this->declare_parameter("serial_timeout", 0.5);  // 秒
  this->declare_parameter("tread", 0.376);
  this->declare_parameter("l_wheel_radius", 0.03858);
  this->declare_parameter("r_wheel_radius", 0.03858);
  this->declare_parameter("reduction_ratio", 20.0);
  this->declare_parameter("encoder_resolution", 30);
  this->declare_parameter("product_id", 0);

  this->get_parameter("odom_frame_id", odom_frame_id_);
  this->get_parameter("base_link_frame_id", base_link_frame_id_);
  this->get_parameter("subscribe_topic_name", subscribe_topic_name);
  this->get_parameter("publish_topic_name", publish_topic_name);
  this->get_parameter("control_frequency", control_frequency);
  this->get_parameter("serial_port", serial_port);
  this->get_parameter("serial_baudrate", serial_baudrate);
  this->get_parameter("cmd_vel_timeout", cmd_vel_timeout_);
  this->get_parameter("serial_timeout", serial_timeout_);
  this->get_parameter("tread", tread);
  this->get_parameter("l_wheel_radius", l_wheel_radius);
  this->get_parameter("r_wheel_radius", r_wheel_radius);
  this->get_parameter("reduction_ratio", reduction_ratio);
  this->get_parameter("encoder_resolution", encoder_resolution);
  this->get_parameter("product_id", product_id);

  RCLCPP_INFO(this->get_logger(), "設定パラメータ");
  RCLCPP_INFO(this->get_logger(), "odom_frame_id: %s", odom_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "base_link_frame_id: %s", base_link_frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "subscribe_topic_name: %s", subscribe_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "publish_topic_name: %s", publish_topic_name.c_str());
  RCLCPP_INFO(this->get_logger(), "control_frequency: %f", control_frequency);
  RCLCPP_INFO(this->get_logger(), "serial_port: %s", serial_port.c_str());
  RCLCPP_INFO(this->get_logger(), "serial_baudrate: %d", serial_baudrate);
  RCLCPP_INFO(this->get_logger(), "cmd_vel_timeout: %f", cmd_vel_timeout_);
  RCLCPP_INFO(this->get_logger(), "serial_timeout: %f", serial_timeout_);
  RCLCPP_INFO(this->get_logger(), "tread: %f", tread);
  RCLCPP_INFO(this->get_logger(), "l_wheel_radius: %f", l_wheel_radius);
  RCLCPP_INFO(this->get_logger(), "r_wheel_radius: %f", r_wheel_radius);
  RCLCPP_INFO(this->get_logger(), "reduction_ratio: %f", reduction_ratio);
  RCLCPP_INFO(this->get_logger(), "encoder_resolution: %d", encoder_resolution);
  RCLCPP_DEBUG(this->get_logger(), "product_id: %d", product_id);

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
  current_odom_.header.frame_id = odom_frame_id_;
  current_odom_.child_frame_id = base_link_frame_id_;

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
  rclcpp::Time current_receive_time = this->get_clock()->now();

  // 1. ボディデータからエンコーダ値を取得
  int32_t current_left_encoder = cugo_ros2_control2::Serial::bin_to_int32(body_data.data() + 0);
  int32_t current_right_encoder = cugo_ros2_control2::Serial::bin_to_int32(body_data.data() + 4);
  RCLCPP_INFO(this->get_logger(), "Encoder: L=%d, R=%d",
      current_left_encoder, current_right_encoder);

  // 2. 状態を更新（Mutexで保護）
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    // 最初のデータ受信時は、前回値として保存するだけ
    if (is_first_serial_data_) {
      prev_left_encoder_ = current_left_encoder;
      prev_right_encoder_ = current_right_encoder;
      last_serial_receive_time_ = current_receive_time;
      is_first_serial_data_ = false;
      return;
    }

    // 2回目以降の受信
    double dt = (current_receive_time - last_serial_receive_time_).seconds();
    if (dt <= 0.0) { // 時間が進んでいない場合は計算しない
      RCLCPP_WARN(this->get_logger(), "dt is zero or negative. Skipping odometry calculation.");
      return;
    }

    // エンコーダの差分を計算
    int32_t diff_l = current_left_encoder - prev_left_encoder_;
    int32_t diff_r = current_right_encoder - prev_right_encoder_;

    // 速度(Twist)を計算
    auto twist = cugo_->calc_twist(diff_l, diff_r, dt);

    // 現在のオドメトリ情報を一時的な構造体にコピー
    cugo_ros2_control2::Odom current_odom_state;
    current_odom_state.x = current_odom_.pose.pose.position.x;
    current_odom_state.y = current_odom_.pose.pose.position.y;
    tf2::Quaternion q(
      current_odom_.pose.pose.orientation.x, current_odom_.pose.pose.orientation.y,
      current_odom_.pose.pose.orientation.z, current_odom_.pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_odom_state.yaw = yaw;

    // 新しいオドメトリを計算
    auto updated_odom_state = cugo_->calc_odom(current_odom_state, twist, dt);

    // 計算結果をメンバ変数に反映
    current_odom_.pose.pose.position.x = updated_odom_state.x;
    current_odom_.pose.pose.position.y = updated_odom_state.y;
    current_odom_.twist.twist.linear.x = twist.linear_x;
    current_odom_.twist.twist.angular.z = twist.angular_z;

    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, updated_odom_state.yaw);
    current_odom_.pose.pose.orientation = tf2::toMsg(q_new);

    // 次の計算のために、今回の値を「前回値」として保存
    prev_left_encoder_ = current_left_encoder;
    prev_right_encoder_ = current_right_encoder;
    last_serial_receive_time_ = current_receive_time;
  }

  // 3. 計算したオドメトリとTFを発行
  publish_odom_and_tf();
  RCLCPP_DEBUG(this->get_logger(), "serial_data_callback() published");
}

void Node::control_loop()
{
  // --- 共有データをローカルにコピー ---
  geometry_msgs::msg::Twist local_cmd_vel;
  rclcpp::Time local_last_cmd_vel_time;
  rclcpp::Time local_last_serial_receive_time;

  // cmd_velにアクセスしてすぐにロック解除
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    local_cmd_vel = latest_cmd_vel_;
    local_last_cmd_vel_time = last_cmd_vel_time_;
    local_last_serial_receive_time = last_serial_receive_time_;
  }

  auto now = this->get_clock()->now();

  // --- 送信処理 ---
  cugo_ros2_control2::SendValue sv;
  sv.product_id = product_id;  // V4/V3iの区別。マイコン内でパラメータを切り替え処理を行う。
  sv.robot_id = 0; // 複数台ロボットを動かしたときに使いたい。今は何もしない。

  if ((now - local_last_cmd_vel_time).seconds() > cmd_vel_timeout_) {
    sv.l_rpm = 0.0f;
    sv.r_rpm = 0.0f;
  } else {
    auto rpm = cugo_->calc_rpm(local_cmd_vel.linear.x, local_cmd_vel.angular.z);
    sv.l_rpm = rpm.l_rpm;
    sv.r_rpm = rpm.r_rpm;
  }
  RCLCPP_DEBUG(this->get_logger(),"command rpm(L/R): %f / %f", sv.l_rpm, sv.r_rpm);
  serial_->write(sv);

  // --- シリアルタイムアウト監視 ---
  if ((now - local_last_serial_receive_time).seconds() > serial_timeout_) {
    RCLCPP_WARN(this->get_logger(), "シリアル通信未達。接続を確認してください。");
    // Picoが機能不全の可能性。速度ゼロのオドメトリを発行して異常を知らせる。
    std::lock_guard<std::mutex> lock(data_mutex_);
    current_odom_.twist.twist.linear.x = 0.0;
    current_odom_.twist.twist.angular.z = 0.0;
    publish_odom_and_tf(); // 既に止まっている位置情報と速度ゼロを定期的に発行
    RCLCPP_DEBUG(this->get_logger(), "control_loop() published");
  }
}

// オドメトリとTFを発行するヘルパー関数
void Node::publish_odom_and_tf()
{
  rclcpp::Time now = this->get_clock()->now();

  // TF送信用 TransformStamped メッセージの作成
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = now;
  t.header.frame_id = odom_frame_id_;
  t.child_frame_id = base_link_frame_id_;

  // OdometryメッセージのPose情報をそのまま使う
  t.transform.translation.x = current_odom_.pose.pose.position.x;
  t.transform.translation.y = current_odom_.pose.pose.position.y;
  t.transform.translation.z = 0.0;
  t.transform.rotation = current_odom_.pose.pose.orientation;
  tf_broadcaster_->sendTransform(t);

  // Odometryメッセージのヘッダを更新して発行
  current_odom_.header.stamp = now;
  odom_pub_->publish(current_odom_);

  tf2::Quaternion q(
    current_odom_.pose.pose.orientation.x,
    current_odom_.pose.pose.orientation.y,
    current_odom_.pose.pose.orientation.z,
    current_odom_.pose.pose.orientation.w);
  double roll, pitch, yaw;
  tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

  RCLCPP_INFO(this->get_logger(), "Odometry: X=%lf, Y=%lf, Orientation=%lf",
      current_odom_.pose.pose.position.x,
      current_odom_.pose.pose.position.y,
      yaw);
  RCLCPP_INFO(this->get_logger(), "Velocity: Linear=%lf, Angular=%lf",
      current_odom_.twist.twist.linear.x,
      current_odom_.twist.twist.angular.z);
}
