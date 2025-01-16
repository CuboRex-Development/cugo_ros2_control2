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
  float config_l_radius, float config_r_radius, float config_tread,
  float config_reduction_ratio, int config_encoder_resolution)
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

RPM CuGo::calc_rpm(float linear_x, float angular_z)
{
  RPM rpm;
  float l_radian = linear_x / l_wheel_radius - tread * angular_z / (2 * l_wheel_radius);
  float r_radian = linear_x / r_wheel_radius + tread * angular_z / (2 * r_wheel_radius);
  rpm.l_rpm = l_radian * 60 / (2 * M_PI);
  rpm.r_rpm = r_radian * 60 / (2 * M_PI);
  return rpm;
}

Twist CuGo::calc_twist(int l_count_diff, int r_count_diff, float dt)
{
  float l_velocity =
    l_count_diff / (encoder_resolution * reduction_ratio) * 2 * l_wheel_radius * M_PI;
  float r_velocity =
    r_count_diff / (encoder_resolution * reduction_ratio) * 2 * r_wheel_radius * M_PI;

  float x_velcity = (l_velocity + r_velocity) / 2;
  float theta_velocity = (r_velocity - l_velocity) / tread;

  Twist twist;
  twist.linear_x = x_velcity / dt;
  twist.angular_z = theta_velocity / dt;
  return twist;
}

Odom CuGo::calc_odom(Odom input_odom, Twist twist, float dt)
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
float CuGo::get_tread()
{
  return tread;
}

float CuGo::get_l_wheel_radius()
{
  return l_wheel_radius;
}

float CuGo::get_r_wheel_radius()
{
  return r_wheel_radius;
}

float CuGo::get_reduction_ratio()
{
  return reduction_ratio;
}

float CuGo::get_encoder_resolution()
{
  return encoder_resolution;
}

void CuGo::set_twist(float linear_x, float angular_z)
{
  recv_twist.linear_x = linear_x;
  recv_twist.angular_z = angular_z;
}

void CuGo::set_difftime_cmdvel(float difftime)
{
  difftime_cmdvel = difftime;
}
