#ifndef CUGO_ROS2_CONTROL2_SERIAL_HPP
#define CUGO_ROS2_CONTROL2_SERIAL_HPP

#include <string>

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
#endif  // CUGO_ROS2_CONTROL2_SERIAL_HPP
