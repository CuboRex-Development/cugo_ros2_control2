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
#ifndef CUGO_ROS2_CONTROL2_SERIAL_HPP
#define CUGO_ROS2_CONTROL2_SERIAL_HPP

#include <string>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <iostream>
#include <algorithm>
#include "cugo_ros2_control2/COBS.h"

namespace cugo_ros2_control2
{

class Serial
{
public:
  Serial(const std::string & port, int baudrate);
  void open();
  void close();
  std::string read();
  void write(const std::string & data);
  std::string encode(const std::string & data);
  std::string decode(const std::string & data);
  float bin_to_float(const std::string & data);
  int bin_to_int(const std::string & data);
  std::string float_to_bin(float value);
  std::string int_to_bin(int value);
  int calc_checksum(const std::string & data);
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_SERIAL_HPP
