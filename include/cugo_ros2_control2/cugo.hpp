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
  double linear_x;
  double angular_z;
};

struct Odom
{
  double x;
  double y;
  double yaw;
};


namespace cugo_ros2_control2
{

class CuGo
{
public:
  CuGo();
  CuGo(
    double config_l_radius, double config_r_radius, double config_tread,
    double config_reduction_ratio, int config_encoder_resolution);
  void set_params();
  RPM calc_rpm(double linear_x, double angular_z);
  Twist calc_twist(int count_diff_l, int count_diff_r, double dt);
  Odom calc_odom(Odom odom, Twist twist, double dt);
  bool check_invalid_value();
  bool check_timeout();
  void initialize();
  int get_mcu_init_value();
  double get_tread();
  double get_l_wheel_radius();
  double get_r_wheel_radius();
  double get_reduction_ratio();
  double get_encoder_resolution();
  void set_twist(double linear_x, double angular_z);
  void set_difftime_cmdvel(double difftime);

private:
  double tread;
  double l_wheel_radius, r_wheel_radius;
  double reduction_ratio;
  int encoder_resolution;
  int count_diff_l, count_diff_r;
  bool serial_timeout, twist_timeout;
  Twist recv_twist;
  double difftime_cmdvel;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_CUGO_HPP
