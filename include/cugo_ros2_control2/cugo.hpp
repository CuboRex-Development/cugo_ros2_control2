#ifndef CUGO_ROS2_CONTROL2_CUGO_HPP
#define CUGO_ROS2_CONTROL2_CUGO_HPP

class CuGo
{
public:
  CuGo();
  void set_params();
  void calc_rpm();
  void calc_twist();
  void calc_odom();
  bool check_invalid_value();
  bool check_timeout();
  void initialize();
  int get_mcu_init_value();
};
#endif  // CUGO_ROS2_CONTROL2_CUGO_HPP
