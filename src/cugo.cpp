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
CuGo::CuGo()
{
  // configによる指定がない場合はV4のパラメータを使用する
  l_wheel_radius = 0.03858f;
  r_wheel_radius = 0.03858f;
  tread = 0.376f;
  reduction_ratio = 20.0f;
  encoder_resolution = 360;
}

CuGo::CuGo(
  double config_l_radius, double config_r_radius, double config_tread,
  double config_reduction_ratio, int config_encoder_resolution)
{
  // configによる指定がある場合はlaunchファイルのパラメータを使用する
  l_wheel_radius = config_l_radius;
  r_wheel_radius = config_r_radius;
  tread = config_tread;
  reduction_ratio = config_reduction_ratio;
  encoder_resolution = config_encoder_resolution;
}


void CuGo::set_params()
{
}

RPM CuGo::calc_rpm(double linear_x, double angular_z)
{
  RPM rpm;
  double l_radian = linear_x / l_wheel_radius - tread * angular_z / (2 * l_wheel_radius);
  double r_radian = linear_x / r_wheel_radius + tread * angular_z / (2 * r_wheel_radius);
  rpm.l_rpm = (float)(l_radian) * 60 / (2 * M_PI);
  rpm.r_rpm = (float)(r_radian) * 60 / (2 * M_PI);
  return rpm;
}

Twist CuGo::calc_twist(int l_count_diff, int r_count_diff, double dt)
{
  double l_velocity =
    l_count_diff / (encoder_resolution * reduction_ratio) * 2 * l_wheel_radius * M_PI;
  double r_velocity =
    r_count_diff / (encoder_resolution * reduction_ratio) * 2 * r_wheel_radius * M_PI;

  double x_velcity = (l_velocity + r_velocity) / 2;
  double theta_velocity = (r_velocity - l_velocity) / tread;

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

bool CuGo::check_invalid_value() {return false;}
bool CuGo::check_timeout() {return false;}
void CuGo::initialize() {}
int CuGo::get_mcu_init_value() {return 0;}
double CuGo::get_tread()
{
  return tread;
}

double CuGo::get_l_wheel_radius()
{
  return l_wheel_radius;
}

double CuGo::get_r_wheel_radius()
{
  return r_wheel_radius;
}

double CuGo::get_reduction_ratio()
{
  return reduction_ratio;
}

double CuGo::get_encoder_resolution()
{
  return encoder_resolution;
}

void CuGo::set_twist(double linear_x, double angular_z)
{
  recv_twist.linear_x = linear_x;
  recv_twist.angular_z = angular_z;
}

void CuGo::set_difftime_cmdvel(double difftime)
{
  difftime_cmdvel = difftime;
}
