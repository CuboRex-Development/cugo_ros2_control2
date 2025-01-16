#ifndef CUGO_ROS2_CONTROL2_CUGO_HPP
#define CUGO_ROS2_CONTROL2_CUGO_HPP
#include <math.h>

struct RPM
{
  float l_rpm;
  float r_rpm;
};

struct Twist
{
  float linear_x;
  float angular_z;
};

struct Odom
{
  float x;
  float y;
  float yaw;
};


namespace cugo_ros2_control2
{

class CuGo
{
public:
  CuGo();
  CuGo(
    float config_l_radius, float config_r_radius, float config_tread,
    float config_reduction_ratio, int config_encoder_resolution);
  void set_params();
  RPM calc_rpm(float linear_x, float angular_z);
  Twist calc_twist(int count_diff_l, int count_diff_r, float dt);
  Odom calc_odom(Odom odom, Twist twist, float dt);
  bool check_invalid_value();
  bool check_timeout();
  void initialize();
  int get_mcu_init_value();
  float get_tread();
  float get_l_wheel_radius();
  float get_r_wheel_radius();
  float get_reduction_ratio();
  float get_encoder_resolution();
  void set_twist(float linear_x, float angular_z);
  void set_difftime_cmdvel(float difftime);

private:
  float tread;
  float l_wheel_radius, r_wheel_radius;
  float reduction_ratio;
  int encoder_resolution;
  int count_diff_l, count_diff_r;
  bool serial_timeout, twist_timeout;
  Twist recv_twist;
  float difftime_cmdvel;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_CUGO_HPP
