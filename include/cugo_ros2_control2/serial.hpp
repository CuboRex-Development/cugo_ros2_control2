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
#include <functional>
#include <vector>
#include <thread>
#include <array>
#include <cstdint>
#include <cstddef>
#include <stdexcept>
//#include "cugo_ros2_control2/COBS.h"

#define PACKET_SIZE 72
#define PACKET_HEADER_SIZE 8
#define PACKET_BODY_SIZE 64

struct SendValue
{
  uint16_t pc_port;
  uint16_t mcu_port;
  float l_rpm;
  float r_rpm;
  // 送信メッセージが増えればここに追加
};

namespace cugo_ros2_control2
{

class Serial
{
public:
  // コールバック関数の型エイリアス
  using DataCallback = std::function<void (const std::vector<uint8_t> &)>;

  Serial();
  ~Serial();
  void open(const std::string & port, int baudrate);
  void close();
  void register_callback(DataCallback callback);
  void start_read();
  //std::string read();
  //void write(const std::string & data);
  void write(const SendValue & sv);

  // パケット関連メソッド
  static std::vector<unsigned char> create_packet(const SendValue & sv);
  static std::vector<unsigned char> encode(const std::vector<unsigned char> & packet_data);
  static std::vector<unsigned char> decode(const std::vector<unsigned char> & received_data);
  static uint16_t calc_checksum(const unsigned char * body_data, size_t body_size);

  // バイナリ変換
  static std::vector<unsigned char> float_to_bin(float value);
  static float bin_to_float(const unsigned char * data);

  static int bin_to_int(const std::string & data);
  static std::string int_to_bin(int value);

  // boostライブラリ
  boost::asio::io_service io_service_;
  boost::asio::serial_port serial_port_;
  std::thread io_thread_;
  std::array<uint8_t, 256> read_buffer_;
  std::vector<uint8_t> read_data_;
  boost::asio::streambuf stream_buffer_; // PacketSerialデコード用の一時バッファとして使うかも？
  DataCallback data_callback_;

private:
  void handle_read(const boost::system::error_code & error, std::size_t bytes_transferred);
  void handle_write(const boost::system::error_code & error, std::size_t bytes_transferred);

  // writeメソッドの実装で使うキュー (オプション)
  // std::deque<std::vector<unsigned char>> write_queue_;
  // std::mutex write_mutex_;
};

} // namespace cugo_ros2_control2
#endif  // CUGO_ROS2_CONTROL2_SERIAL_HPP
