#include "cugo_ros2_control2/serial.hpp"

Serial::Serial(const std::string & port, int baudrate) {}

void Serial::open() {}
void Serial::close() {}
std::string Serial::read() {return "";}
void Serial::write(const std::string & data) {}
std::string Serial::encode(const std::string & data) {return data;}
std::string Serial::decode(const std::string & data) {return data;}
float Serial::bin_to_float(const std::string & data) {return 0.0f;}
int Serial::bin_to_int(const std::string & data) {return 0;}
std::string Serial::float_to_bin(float value) {return "";}
std::string Serial::int_to_bin(int value) {return "";}
int Serial::calc_checksum(const std::string & data) {return 0;}
