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
: serial_port_(io_context_),
  work_guard_(boost::asio::make_work_guard(io_context_.get_executor()))
{
  io_thread_ = std::thread([this]() {io_context_.run();});
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
            io_context_.run();
            std::cout << "[Serial INFO] io_context_ finished." << std::endl;
          } catch (const std::exception & e) {
            std::cerr << "[Serial FATAL] io_context_ exception: " << e.what() << std::endl;
          }
        });
    }
    start_read(); // ポートが開いたら読み取りを開始
  } catch (const boost::system::system_error & e) {
    std::cerr << "[Serial ERROR] Error opening serial port " << port << ": " << e.what() <<
      std::endl;
    std::cout << "[Serial INFO] シリアルポートの指定が正しいか、読み書き権限があるか確認してください" << std::endl;
    throw e;
  }
}

void Serial::close()
{
  // io_context_に与えていたダミーワークをリセットしてrun()から脱出
  work_guard_.reset();

  if (!io_context_.stopped()) {
    io_context_.stop();
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

void Serial::register_callback(DataCallback callback)
{
  data_callback_ = callback;
}

void Serial::start_read()
{
  serial_port_.async_read_some(
      boost::asio::buffer(raw_read_buffer_),
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
  // --- 1. エラーチェック ---
  if (error) {
    if (error == boost::asio::error::operation_aborted) {
      // ポートが閉じられたなどの正常な終了
      std::cout << "[Serial INFO][handle_read] Read operation aborted." << std::endl;
    } else {
      // その他の予期せぬエラー
      std::cerr << "[Serial ERROR][handle_read] Read error: " << error.message() << std::endl;
    }
    return; // エラー時はループを継続しない
  }

  // --- 2. 受信データをパケット組み立て用バッファ(packet_buffer_)に追加 ---
  if (bytes_transferred > 0) {
    packet_buffer_.insert(
        packet_buffer_.end(),
        raw_read_buffer_.begin(),
        raw_read_buffer_.begin() + bytes_transferred
    );
  }

  // --- 3. バッファからパケットを探索・処理するループ ---
  while (true) {
    // デリミタ(0x00)をバッファの先頭から探す
    auto it = std::find(packet_buffer_.begin(), packet_buffer_.end(), 0x00);

    // デリミタが見つからなければ、まだパケットが全部届いていない。
    // ループを抜けて、次のデータ受信を待つ。
    if (it == packet_buffer_.end()) {
      break;
    }

    // --- 4. 完全なパケットが見つかったので、処理する ---
    // デリミタの位置+1までのデータを1つのパケットとして抽出
    std::vector<unsigned char> received_packet(packet_buffer_.begin(), it + 1);

    // 処理済みの部分を組み立て用バッファから削除
    packet_buffer_.erase(packet_buffer_.begin(), it + 1);

    try {
      // パケットをデコード (COBS -> 生パケット)
      std::vector<unsigned char> decoded_packet = decode(received_packet);

      // パケットを検証 (サイズやチェックサム)
      if (decoded_packet.size() != 72) {
        std::cerr << "[Serial WARN] Decoded packet size is not 72 bytes: "
                  << decoded_packet.size() << std::endl;
      } else {
        unsigned char * body_ptr = &decoded_packet[8];
        uint16_t received_checksum = *reinterpret_cast<uint16_t *>(decoded_packet.data() + 6);
        uint16_t calculated_checksum = calc_checksum(body_ptr, 64);

        if (received_checksum == calculated_checksum) {
          // 検証成功！Nodeに通知
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
  } // パケット探索ループの終わり

  // --- 5. 次の非同期読み込みを開始 ---
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
  for (size_t i = 0; i < body_size; i += 2) {
    uint16_t word =
      (static_cast<uint16_t>(body_data[i + 1]) << 8) | static_cast<uint16_t>(body_data[i]);
    sum += word;
  }
  if (sum >> 16) {
    sum = (sum & 0xFFFF) + (sum >> 16);
  }
  uint16_t final_sum_16bit = static_cast<uint16_t>(sum);
  uint16_t checksum = ~final_sum_16bit;
  return checksum;
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
}


std::vector<unsigned char> Serial::encode(const std::vector<unsigned char> & raw_packet)
{
  const size_t source_size = raw_packet.size();
  if (source_size == 0) {
    // 空のパケットはコード(0x01)と終端マーカー(0x00)のみ
    return {0x01, 0x00};
  }

  // 必要な最大バッファサイズを計算
  const size_t max_encoded_size = source_size + (source_size / 254) + 1;
  std::vector<unsigned char> encoded_packet(max_encoded_size);

  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  unsigned char code = 1;

  while (read_index < source_size) {
    if (raw_packet[read_index] == 0) {
      encoded_packet[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    } else {
      encoded_packet[write_index++] = raw_packet[read_index++];
      code++;

      if (code == 0xFF) {
        encoded_packet[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
  }

  encoded_packet[code_index] = code;

  // 実際のエンコードサイズにリサイズ
  encoded_packet.resize(write_index);

  // パケット終端マーカーを追加
  encoded_packet.push_back(0x00);

  return encoded_packet;
}

std::vector<unsigned char> Serial::decode(const std::vector<unsigned char> & received_packet)
{
  if (received_packet.empty()) {
    return {}; // 空の入力は空の出力
  }

  // 終端マーカーを除いた、エンコードされた本体の部分を抽出
  std::vector<unsigned char> encoded_body;
  if (received_packet.back() == 0x00) {
    // 終端マーカーが0x00の場合（COBS）
    if (received_packet.size() > 1) {
      encoded_body.assign(received_packet.begin(), received_packet.end() - 1);
    }
  } else {
    // 予期せぬデータの場合
    throw std::runtime_error("Decode Error: Packet does not end with 0x00 marker.");
  }

  if (encoded_body.empty()) {
    return {};
  }

  const size_t source_size = encoded_body.size();
  std::vector<unsigned char> decoded_packet;
  decoded_packet.reserve(source_size); // 事前にメモリ確保

  size_t read_index = 0;

  while (read_index < source_size) {
    unsigned char code = encoded_body[read_index];
    if (code == 0) {
      throw std::runtime_error("Decode Error: Zero byte found in encoded data body.");
    }

    if (read_index + code > source_size && code != 1) {
      throw std::runtime_error("Decode Error: Data is shorter than specified by code.");
    }

    read_index++;

    for (unsigned char i = 1; i < code; i++) {
      decoded_packet.push_back(encoded_body[read_index++]);
    }

    // 最後のブロックでなければ0を追加する
    if (code != 0xFF && read_index < source_size) {
      decoded_packet.push_back(0);
    }
  }

  return decoded_packet;
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
