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
#include <boost/asio/io_context.hpp>
#include <boost/asio/executor_work_guard.hpp>
#include <iostream>
#include <algorithm>
#include <functional>
#include <vector>
#include <thread>
#include <array>
#include <cstdint>
#include <cstddef>
#include <stdexcept>

#define PACKET_SIZE 72
#define PACKET_HEADER_SIZE 8
#define PACKET_BODY_SIZE 64

namespace cugo_ros2_control2
{

struct SendValue
{
  uint16_t product_id;
  uint16_t robot_id;
  float l_rpm;
  float r_rpm;
  // 送信メッセージが増えればここに追加
};


class Serial
{
public:
  // コールバック関数の型エイリアス
  using DataCallback = std::function<void (const std::vector<uint8_t> &)>;

  Serial();
  ~Serial();
  void open(const std::string & port, int baudrate);
  void close();
  void start_read();
  void register_callback(DataCallback callback);
  void write(const SendValue & sv);

  // パケット関連メソッド
  static std::vector<unsigned char> create_packet(const SendValue & sv);
  static std::vector<unsigned char> encode(const std::vector<unsigned char> & raw_packet);
  static std::vector<unsigned char> decode(const std::vector<unsigned char> & encoded_packet);
  static uint16_t calc_checksum(const unsigned char * body_data, size_t body_size);

  // バイナリ変換
  static std::vector<unsigned char> float_to_bin(float value);
  static float bin_to_float(const unsigned char * data);
  static std::vector<unsigned char> int32_to_bin(int32_t value);
  static int32_t bin_to_int32(const unsigned char * data);

  // boostライブラリ
  boost::asio::io_context io_context_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  std::array<uint8_t, 256> raw_read_buffer_;
  std::vector<uint8_t> packet_buffer_;
  //boost::asio::streambuf stream_buffer_;
  DataCallback data_callback_;

private:
  void handle_read(const boost::system::error_code & error, std::size_t bytes_transferred);
  void handle_write(const boost::system::error_code & error, std::size_t bytes_transferred);
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_SERIAL_HPP
