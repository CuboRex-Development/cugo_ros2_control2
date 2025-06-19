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
  //CuGo();
  CuGo(
    double config_l_radius, double config_r_radius, double config_tread,
    double config_reduction_ratio, int config_encoder_resolution);
  //void set_params();
  RPM calc_rpm(double linear_x, double angular_z);
  Twist calc_twist(int count_diff_l, int count_diff_r, double dt);
  Odom calc_odom(Odom odom, Twist twist, double dt);
  //bool check_invalid_value();
  //bool check_timeout();
  //void initialize();
  //int get_mcu_init_value();
  //double get_tread();
  //double get_l_wheel_radius();
  //double get_r_wheel_radius();
  //double get_reduction_ratio();
  //double get_encoder_resolution();
  //void set_twist(double linear_x, double angular_z);
  //void set_difftime_cmdvel(double difftime);

private:
  // 物理パラメータのみをメンバ変数として保持
  double WHEEL_RADIUS_L_;
  double WHEEL_RADIUS_R_;
  double TREAD_;
  double REDUCTION_RATIO_;
  int ENCODER_RESOLUTION_;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_CUGO_HPP
