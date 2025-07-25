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

#include "cugo_ros2_control2/cugo.hpp"
#include <iostream>

using namespace cugo_ros2_control2;
CuGo::CuGo(
  double config_l_radius, double config_r_radius, double config_tread,
  double config_reduction_ratio, int config_encoder_resolution)
{
  WHEEL_RADIUS_L_ = config_l_radius;
  WHEEL_RADIUS_R_ = config_r_radius;
  TREAD_ = config_tread;
  REDUCTION_RATIO_ = config_reduction_ratio;
  ENCODER_RESOLUTION_ = config_encoder_resolution;
}

RPM CuGo::calc_rpm(double linear_x, double angular_z)
{
  RPM rpm;
  double l_radian = linear_x / WHEEL_RADIUS_L_ - TREAD_ * angular_z / (2 * WHEEL_RADIUS_L_);
  double r_radian = linear_x / WHEEL_RADIUS_R_ + TREAD_ * angular_z / (2 * WHEEL_RADIUS_R_);
  rpm.l_rpm = (float)(l_radian) * 60 / (2 * M_PI);
  rpm.r_rpm = (float)(r_radian) * 60 / (2 * M_PI);
  return rpm;
}

Twist CuGo::calc_twist(int l_count_diff, int r_count_diff, double dt)
{
  double l_velocity =
    l_count_diff / (ENCODER_RESOLUTION_ * REDUCTION_RATIO_) * 2 * WHEEL_RADIUS_L_ * M_PI;
  double r_velocity =
    r_count_diff / (ENCODER_RESOLUTION_ * REDUCTION_RATIO_) * 2 * WHEEL_RADIUS_R_ * M_PI;

  double x_velcity = (l_velocity + r_velocity) / 2;
  double theta_velocity = (r_velocity - l_velocity) / TREAD_;

  Twist twist;
  twist.linear_x = x_velcity / dt;
  twist.angular_z = theta_velocity / dt;
  return twist;
}

Odom CuGo::calc_odom(Odom input_odom, Twist twist, double dt)
{
  Odom output_odom;
  output_odom.x = 0.0;
  output_odom.y = 0.0;
  output_odom.yaw = 0.0;
  output_odom.yaw = input_odom.yaw + twist.angular_z * dt;
  output_odom.x = input_odom.x + twist.linear_x * dt * cos(output_odom.yaw);
  output_odom.y = input_odom.y + twist.linear_x * dt * sin(output_odom.yaw);
  return output_odom;
}
