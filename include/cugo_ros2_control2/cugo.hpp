#ifndef CUGO_ROS2_CONTROL2_CUGO_HPP
#define CUGO_ROS2_CONTROL2_CUGO_HPP

#include <math.h>

struct RPM
{
  float l_rpm;
  float r_rpm;
};

namespace cugo_ros2_control2
{

class CuGo
{
public:
  CuGo();
  CuGo(float config_l_radius, float config_r_radius, float config_tread);
  void set_params();
  RPM calc_rpm(float linear_x, float angular_z);
  void calc_twist();
  void calc_odom();
  bool check_invalid_value();
  bool check_timeout();
  void initialize();
  int get_mcu_init_value();

private:
  float tread;
  float l_wheel_radius, r_wheel_radius;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_CUGO_HPP
