#include "cugo_ros2_control2/cugo.hpp"

using namespace cugo_ros2_control2;
CuGo::CuGo()
{
  // configによる指定がない場合はV4のパラメータを使用する
  l_wheel_radius = 0.03858f;
  r_wheel_radius = 0.03858f;
  tread = 0.376f;
}

CuGo::CuGo(float config_l_radius, float config_r_radius, float config_tread)
{
  // configによる指定がある場合はlaunchファイルのパラメータを使用する
  l_wheel_radius = config_l_radius;
  r_wheel_radius = config_r_radius;
  tread = config_tread;
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

void CuGo::calc_twist() {}
void CuGo::calc_odom() {}
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
