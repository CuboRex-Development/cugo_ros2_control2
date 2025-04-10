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

Serial::Serial()
: serial_port_(io_service_)
{
  io_thread_ = std::thread([this]() {io_service_.run();});
  std::cout << "[Serial INFO] Serial object created." << std::endl;
}

Serial::~Serial()
{
  std::cout << "[Serial INFO] Destroying Serial object..." << std::endl;
  close();
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  std::cout << "[Serial INFO] Serial object destroyed." << std::endl;
}

void Serial::open(const std::string & port, int baudrate)
{
  if (serial_port_.is_open()) {
    std::cerr << "[Serial WARN] Serial port " << port << " alrady open." << std::endl;
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

    std::cout << "[Serial INFO] Serial port " << port << " opened with baudrate " << baudrate <<
      std::endl;

    // I/O スレッド開始 (エラーハンドリングを少し追加)
    if (!io_thread_.joinable()) {
      io_thread_ = std::thread(
        [this]() {
          try {
            io_service_.run();
            std::cout << "[Serial INFO] io_service finished." << std::endl;
          } catch (const std::exception & e) {
            std::cerr << "[Serial FATAL] io_service exception: " << e.what() << std::endl;
          }
        });
    }
    // start_read(); // ポートが開いたら読み取りを開始
  } catch (const boost::system::system_error & e) {
    std::cerr << "[Serial ERROR] Error opening serial port " << port << ": " << e.what() <<
      std::endl;
    throw e;
  }
}

void Serial::close()
{
  if (!io_service_.stopped()) {
    io_service_.stop();
  }

  if (serial_port_.is_open()) {
    boost::system::error_code ec;
    serial_port_.close(ec);
    if (ec) {
      std::cerr << "[Serial ERROR] Error closing serial port: " << ec.message() << std::endl;
    } else {
      std::cout << "[Serial INFO] Serial port closed." << std::endl;
    }
  }
}

// 未テスト
void Serial::register_callback(DataCallback callback)
{
  data_callback_ = callback;
}

// 未テスト
void Serial::start_read()
{
  serial_port_.async_read_some(
    boost::asio::buffer(read_buffer_),
    boost::bind(
      &Serial::handle_read, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void Serial::write(const SendValue & sv) // 仮実装
{
  // TODO:
  // 1. create_packet(sv) でパケット生成
  // 2. encode(packet) で PacketSerial エンコード
  // 3. boost::asio::async_write で送信
}

uint16_t Serial::calc_checksum(const unsigned char * body_data, size_t body_size)
{
  if (body_size % 2 != 0) {
    std::cerr << "[Serial ERROR] Checksum calculation requires even body size, but got " <<
      body_size << std::endl;
    return 0; // エラーを示すために 0 を返す (呼び出し側で要チェック)
  }

  uint32_t sum = 0;
  std::cout << "[Serial DEBUG] Data: ";
  for (size_t i = 0; i < body_size; i += 2) {
    uint16_t word =
      (static_cast<uint16_t>(body_data[i + 1]) << 8) | static_cast<uint16_t>(body_data[i]);
    printf(
      "  word[%zu]: data[i]=0x%02X, data[i+1]=0x%02X -> word=0x%04X, current sum=0x%08X\n",
      i / 2, body_data[i], body_data[i + 1], word, sum);
    sum += word;
  }
  std::cout << std::endl;
  printf("[Serial DEBUG] sum (before carry): 0x%08X\n", sum);
  if (sum >> 16) {
    sum = (sum & 0xFFFF) + (sum >> 16);
  }
  printf("[Serial DEBUG] sum (after carry): 0x%08X\n", sum);
  uint16_t final_sum_16bit = static_cast<uint16_t>(sum);
  uint16_t checksum = ~final_sum_16bit;
  printf("[Serial DEBUG] checksum (16bit): 0x%04X\n", checksum);
  return checksum;
  //return static_cast<uint16_t>(~sum);
}

std::vector<unsigned char> Serial::create_packet(const SendValue & sv) // 仮実装
{
  // 仮実装 (または後で実装)
  std::vector<unsigned char> packet(PACKET_SIZE, 0);
  // TODO: ヘッダ設定、ボディ設定、チェックサム計算と設定
  return packet;
}

/*
std::vector<unsigned char> create_packet(const SendValue & sv)
{
  // 内部で packet[PACKET_SIZE] の固定長バッファを作成し、
  // ヘッダー・ボディの各フィールドを memcpy() で設定した上で、
  // チェックサム計算を行い、生成したパケットを std::vector<unsigned char> として返す
  unsigned char packet[PACKET_SIZE];
  std::memset(packet, 0, PACKET_SIZE);

  // ヘッダー作成
  uint16_t pc_port_net = htons(static_cast<uint16_t>(sv.pc_port));
  uint16_t mcu_port_net = htons(static_cast<uint16_t>(sv.mcu_port));
  uint16_t length_net = htons(static_cast<uint16_t>(PACKET_SIZE));
  uint16_t checksum = 0;

  std::memcpy(packet, &pc_port_net, sizeof(pc_port_net));
  std::memcpy(packet + 2, &mcu_port_net, sizeof(mcu_port_net));
  std::memcpy(packet + 4, &length_net, sizeof(length_net));
  std::memcpy(packet + 6, &checksum, sizeof(checksum)); // 2バイトのchecksumに注意

  // ボディ作成：先頭8バイトに2つのfloat（l_rpm と r_rpm）をセット
  std::memcpy(packet + PACKET_HEADER_SIZE, &sv.l_rpm, sizeof(float));
  std::memcpy(packet + PACKET_HEADER_SIZE + 4, &sv.r_rpm, sizeof(float));
  // （残りは既に0で初期化済み）

  // チェックサム計算（チェックサムフィールドは 0 として計算）
  //std::string packet_str(reinterpret_cast<char*>(packet), PACKET_SIZE);
  //checksum = static_cast<uint8_t>(calc_checksum(packet_str));
  //std::memcpy(packet + 6, &checksum, sizeof(checksum));

  // 生成した固定長バイナリパケットを vector にコピーして返す
  return std::vector<unsigned char>(packet, packet + PACKET_SIZE);
}
*/

// 未テスト
void Serial::handle_read(const boost::system::error_code & error, std::size_t bytes_transferred)
{
  // TODO: ここに非同期読み取り完了後の処理を実装する
  //       (エラーチェック、デコード、コールバック呼び出し、次の読み取り開始など)

  // とりあえず、何もしないか、簡単なメッセージを標準エラー出力に出力しておく
  if (error) {
    // エラー発生時の仮処理
    std::cerr << "[Serial DEBUG][handle_read] Read error or aborted: " << error.message() <<
      std::endl;
  } else {
    // 正常受信時の仮処理
    std::cout << "[Serial DEBUG][handle_read] Received " << bytes_transferred <<
      " bytes (needs processing)." << std::endl;
  }

  // 本来はここで次の読み取りを開始する start_read() を呼ぶことが多いが、
  // まずはリンクエラー解消のため、これだけでもOK
}

/* old 消す
void Serial::handle_read(const boost::system::error_code & error, std::size_t bytes_transferred)
{
  if (!error) {
    std::istream is(&buffer_);
    std::string line;
    std::getline(is, line);
    std::cout << "Received: " << line << std::endl;

    // 必要に応じてデータコールバックを呼び出す
    if (data_callback_) {
      std::vector<uint8_t> data(line.begin(), line.end());
      data_callback_(data);
    }

    // 次の読み取りを開始
    boost::asio::async_read_until(
      serial_port_, buffer_, '\n',
      boost::bind(
        &Serial::handle_read, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
  } else {
    std::cerr << "Error in handle_read: " << error.message() << std::endl;
  }
}
*/


// 未テスト
/*
void Serial::handle_write(const boost::system::error_code & error, std::size_t bytes_transferred)
{
  if (!error) {
    std::cout << "Successfully wrote " << bytes_transferred << " bytes." << std::endl;
  } else {
    std::cerr << "Error in handle_write: " << error.message() << std::endl;
  }
}
*/

/*
std::string Serial::encode(const std::string & data)
{
  return data;
}
*/

//std::string Serial::decode(const std::string & data) {return data;}
float Serial::bin_to_float(const std::string & data) {return 0.0f;}
int Serial::bin_to_int(const std::string & data) {return 0;}
std::string Serial::float_to_bin(float value) {return "";}
std::string Serial::int_to_bin(int value) {return "";}
//int Serial::calc_checksum(const std::string & data) {return 0;}
