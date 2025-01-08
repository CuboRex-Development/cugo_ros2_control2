#ifndef CUGO_ROS2_CONTROL2_HPP
#define CUGO_ROS2_CONTROL2_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

namespace cugo_ros2_control2
{

class Node : public rclcpp::Node
{
public:
  Node();
  geometry_msgs::msg::Twist last_cmd_vel;
  //void set_params();
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_HPP_
