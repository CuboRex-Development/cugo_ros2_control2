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

#include "cugo_ros2_control2/serial.hpp"

using namespace cugo_ros2_control2;

Serial::Serial(const std::string & port, int baudrate)
: serial_port_(io_service_)
{
  open(port, baudrate);
}

Serial::~Serial()
{
  close();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
}

void Serial::open(const std::string & port, int baudrate)
{
  if (serial_port_.is_open()) {
    std::cerr << "Serial port already open." << std::endl;
    return;
  }

  try {
    serial_port_.open(port);
    serial_port_.set_option(
      boost::asio::serial_port_base::baud_rate(baudrate));
    serial_port_.set_option(
      boost::asio::serial_port_base::character_size(8));
    serial_port_.set_option(
      boost::asio::serial_port_base::parity(
        boost::asio::serial_port_base::parity::none));
    serial_port_.set_option(
      boost::asio::serial_port_base::stop_bits(
        boost::asio::serial_port_base::stop_bits::one));
    serial_port_.set_option(
      boost::asio::serial_port_base::flow_control(
        boost::asio::serial_port_base::flow_control::none));

    std::cout << "Serial port " << port << " opened with baudrate " << baudrate << std::endl;
  } catch (const boost::system::system_error & e) {
    std::cerr << "Error opening serial port: " << e.what() << std::endl;
    throw e;
  }

}

void Serial::close()
{

}

std::string Serial::read()
{
  return "";
}

void Serial::write(const std::string & data)
{

}

std::string Serial::encode(const std::string & data)
{
  return data;
}

std::string Serial::decode(const std::string & data) {return data;}
float Serial::bin_to_float(const std::string & data) {return 0.0f;}
int Serial::bin_to_int(const std::string & data) {return 0;}
std::string Serial::float_to_bin(float value) {return "";}
std::string Serial::int_to_bin(int value) {return "";}
int Serial::calc_checksum(const std::string & data) {return 0;}
