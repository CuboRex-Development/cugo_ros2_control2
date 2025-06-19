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

    // I/O スレッド開始 (エラーハンドリング)
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
    start_read(); // ポートが開いたら読み取りを開始
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
  // PacketSerial の区切り文字(0xC0)が見つかるまで、データを非同期で読み込む
  // データは stream_buffer_ に溜め込まれる
  boost::asio::async_read_until(
      serial_port_,
      stream_buffer_,
    (unsigned char)0xC0,   // この区切り文字が見つかったら handle_read を呼ぶ
      boost::bind(
        &Serial::handle_read,
        this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred
      )
  );
}

void Serial::handle_read(const boost::system::error_code & error, std::size_t bytes_transferred)
{
  // --- エラーチェック ---
  if (error) {
    if (error == boost::asio::error::operation_aborted) {
      std::cout << "[Serial INFO][handle_read] Read operation aborted." << std::endl;
    } else {
      std::cerr << "[Serial ERROR][handle_read] Read error: " << error.message() << std::endl;
    }
    return; // エラー時はループを継続しない
  }

  // --- データ処理 ---
  if (bytes_transferred > 0) {
    // 1. バッファから受信パケットを抽出
    std::istream is(&stream_buffer_);
    std::vector<unsigned char> received_packet(bytes_transferred);
    is.read(reinterpret_cast<char *>(received_packet.data()), bytes_transferred);

    try {
      // 2. パケットをデコード (PacketSerial -> 生パケット)
      std::vector<unsigned char> decoded_packet = decode(received_packet);

      // 3. パケットを検証
      if (decoded_packet.size() != 72) {
        std::cerr << "[Serial WARN] Decoded packet size is not 72 bytes: "
                  << decoded_packet.size() << std::endl;
      } else {
        unsigned char * body_ptr = &decoded_packet[8];
        uint16_t received_checksum = *reinterpret_cast<uint16_t *>(decoded_packet.data() + 6); // TODO:bin_to_int16の作成
        uint16_t calculated_checksum = calc_checksum(body_ptr, 64);

        if (received_checksum == calculated_checksum) {
          // 4. 検証成功！Nodeに通知
          if (data_callback_) {
            std::vector<unsigned char> body_data(body_ptr, body_ptr + 64);
            data_callback_(body_data);
          }
        } else {
          std::cerr << "[Serial WARN] Checksum mismatch!" << std::endl;
        }
      }
    } catch (const std::runtime_error & e) {
      std::cerr << "[Serial WARN] Decode error: " << e.what() << std::endl;
    }
  }

  // --- 次の読み取りを開始 ---
  // ★重要★ この関数を再度呼び出すことで、受信ループを継続する
  start_read();
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

std::vector<unsigned char> Serial::create_packet(const SendValue & sv)
{
  // 72バイトのパケット領域を確保し、0で初期化
  std::vector<unsigned char> packet(72, 0);

  // --- ボディの作成 ---
  // ボディ部分へのポインタを取得
  unsigned char * body_ptr = packet.data() + 8; // ヘッダ(8バイト)の後ろ
  // RPM値をボディにコピー
  memcpy(body_ptr + 0, &sv.l_rpm, sizeof(float)); // ボディの0バイト目から
  memcpy(body_ptr + 4, &sv.r_rpm, sizeof(float)); // ボディの4バイト目から
  // 残りのボディは0で初期化済み

  // --- ヘッダの作成 ---
  // ヘッダ部分へのポインタを取得
  unsigned char * header_ptr = packet.data();
  uint16_t length = 72;
  // ヘッダに各値をコピー
  memcpy(header_ptr + 0, &sv.pc_port, sizeof(uint16_t));
  memcpy(header_ptr + 2, &sv.mcu_port, sizeof(uint16_t));
  memcpy(header_ptr + 4, &length, sizeof(uint16_t));

  // ボディデータからチェックサムを計算
  uint16_t checksum = calc_checksum(body_ptr, 64);
  // 計算したチェックサムをヘッダにコピー
  memcpy(header_ptr + 6, &checksum, sizeof(uint16_t));

  return packet;
}

void Serial::write(const SendValue & sv)
{
  // Step 1: 送信するデータ構造体から、72バイトの生パケットを生成する
  std::vector<unsigned char> raw_packet = create_packet(sv);

  // Step 2: 生パケットを PacketSerial 形式にエンコードする
  std::vector<unsigned char> encoded_packet = encode(raw_packet);

  // Step 3: エンコードされたパケットを非同期で送信する
  boost::asio::async_write(
      serial_port_,
      boost::asio::buffer(encoded_packet),
      boost::bind(
        &Serial::handle_write,
        this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred
      )
  );
}

void Serial::handle_write(const boost::system::error_code & error, size_t/* bytes_transferred*/)
{
  if (error) {
    // ポートを閉じたことによる正常な中断はエラーとして扱わない
    if (error == boost::asio::error::operation_aborted) {
      std::cout << "[Serial INFO][handle_write] Write operation aborted." << std::endl;
    } else {
      // その他の書き込みエラー
      std::cerr << "[Serial ERROR][handle_write] Write error: " << error.message() << std::endl;
    }
    return;
  }

  // 正常に送信完了した場合、デバッグ用に何か表示しても良い (任意)
  // std::cout << "[Serial DEBUG][handle_write] Sent " << bytes_transferred << " bytes." << std::endl;
}


std::vector<unsigned char> Serial::encode(const std::vector<unsigned char> & raw_packet)
{
  std::vector<unsigned char> encoded;

  for (const auto & byte : raw_packet) {
    if (byte == 0xC0) {         // 区切り文字 (END)
      encoded.push_back(0xDB); // エスケープ文字 (ESC)
      encoded.push_back(0xDC); // ESC_END
    } else if (byte == 0xDB) {  // エスケープ文字 (ESC)
      encoded.push_back(0xDB); // エスケープ文字 (ESC)
      encoded.push_back(0xDD); // ESC_ESC
    } else {
      encoded.push_back(byte);
    }
  }

  // 末尾に区切り文字を追加
  encoded.push_back(0xC0);

  return encoded;
}

std::vector<unsigned char> Serial::decode(const std::vector<unsigned char> & encoded_packet)
{
  // 入力データの基本的なチェック
  if (encoded_packet.empty() || encoded_packet.back() != 0xC0) {
    throw std::runtime_error("Invalid packet: missing END character or empty packet.");
  }

  std::vector<unsigned char> decoded;
  // 末尾の区切り文字(0xC0)を除いた範囲をループ
  for (size_t i = 0; i < encoded_packet.size() - 1; ++i) {
    if (encoded_packet[i] == 0xDB) { // エスケープ文字
                                     // 次のバイトをチェックするため、インデックスを一つ進める
      i++;
      if (i >= encoded_packet.size() - 1) { // エスケープ文字の直後が終端だった場合
        throw std::runtime_error("Invalid packet: trailing ESC character.");
      }

      if (encoded_packet[i] == 0xDC) { // ESC_END
        decoded.push_back(0xC0);
      } else if (encoded_packet[i] == 0xDD) { // ESC_ESC
        decoded.push_back(0xDB);
      } else {
        // 不正なエスケープシーケンス
        throw std::runtime_error("Invalid packet: invalid escape sequence.");
      }
    } else {
      // 通常のデータ
      decoded.push_back(encoded_packet[i]);
    }
  }
  return decoded;
}

std::vector<unsigned char> Serial::float_to_bin(float value)
{
  std::vector<unsigned char> bytes(sizeof(float));
  memcpy(bytes.data(), &value, sizeof(float));
  return bytes;
}

float Serial::bin_to_float(const unsigned char * data)
{
  float value;
  memcpy(&value, data, sizeof(float));
  return value;
}

// int32_t -> bin
std::vector<unsigned char> Serial::int32_to_bin(int32_t value)
{
  std::vector<unsigned char> bytes(sizeof(int32_t)); // 4バイト確保
  // value のメモリ上の表現を bytes ベクターにコピー
  memcpy(bytes.data(), &value, sizeof(int32_t));
  return bytes;
}

// bin -> int32_t
int32_t Serial::bin_to_int32(const unsigned char * data)
{
  int32_t value;
  // data が指すメモリから sizeof(int32_t) バイトを value にコピー
  memcpy(&value, data, sizeof(int32_t));
  return value;
}
