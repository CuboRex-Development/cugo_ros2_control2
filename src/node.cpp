#include "cugo_ros2_control2/node.hpp"

using namespace cugo_ros2_control2;
Node::Node()
: rclcpp::Node("cugo_ros2_control2")
{
  RCLCPP_INFO(this->get_logger(), "cugo_ros2_control2 has started.");

  // launchファイルからパラメータを取得
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

  CuGo cugo{l_wheel_radius, r_wheel_radius, tread, reduction_ratio, encoder_resolution};
  //Serial serial{hoge,piyo,fuga};

  // TODO: topic名を変更
  cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 10,
    std::bind(&Node::cmd_vel_callback, this, std::placeholders::_1));

  // TODO: topic名を変更
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

  // メインループ
  control_timer = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
    std::bind(&Node::control, this)
  );

  // タイムアウトをチェック（1Hz）
  check_timeout_timer = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&Node::notify_message, this)
  );

}

void Node::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  last_cmd_vel.linear_x = msg->linear.x;
  last_cmd_vel.angular_z = msg->angular.z;
  recvtime_cmdvel = this->get_clock()->now();
  RCLCPP_DEBUG(
    this->get_logger(), "Received /cmd_vel: linear_x = %f, angular_z = %f",
    last_cmd_vel.linear_x, last_cmd_vel.angular_z);
  RCLCPP_DEBUG(this->get_logger(), "recvtime_cmdvel update: %f", recvtime_cmdvel.seconds());
}

bool Node::is_timeout(float current_time, float prev_time, float timeout_duration)
{
  return check_difftime(current_time, prev_time) >= timeout_duration;
}

float Node::check_difftime(float current_time, float prev_time)
{
  return current_time - prev_time;
}

void Node::control()
{
  RCLCPP_DEBUG(this->get_logger(), "10Hz Job");
}

void Node::notify_message()
{
  RCLCPP_DEBUG(this->get_logger(), "1Hz Job");
}

/*
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
*/
