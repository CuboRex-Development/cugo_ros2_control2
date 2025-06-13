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

#include <gtest/gtest.h>
//#include "cugo_ros2_control2/node.hpp"
#include "cugo_ros2_control2/serial.hpp"
#include <vector>
#include <cstdint>

using namespace cugo_ros2_control2;

// テストフィクスチャ
class SerialTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のSerialインスタンスを初期化
    test_serial = std::make_shared<Serial>();
  }

  void TearDown() override
  {
    // クリーンアップ
    if (test_serial && test_serial->serial_port_.is_open()) {
      test_serial->close();
    }
  }

  std::shared_ptr<Serial> test_serial;
};

// USBとRaspberryPiPicoを接続してcolcon build / colcon testすること
TEST_F(SerialTest, test_open)
{
  // シリアルポートを開くテスト
  EXPECT_NO_THROW(
  {
    test_serial->open("/dev/ttyACM0", 115200);
  });
  EXPECT_TRUE(test_serial->serial_port_.is_open());

  // 存在しないポートを開こうとすると例外を投げるテスト
  test_serial->close();
  EXPECT_THROW(
  {
    test_serial->open("/dev/null", 115200);
  }, boost::system::system_error);
}

TEST_F(SerialTest, test_close)
{
  // ポートを開いてから閉じるテスト
  EXPECT_NO_THROW(
  {
    test_serial->open("/dev/ttyACM0", 115200);
  });
  EXPECT_TRUE(test_serial->serial_port_.is_open());

  EXPECT_NO_THROW(
  {
    test_serial->close();
  });
  EXPECT_FALSE(test_serial->serial_port_.is_open());
}

// Serial::calc_checksum のテストケース (int32_t データパターン)
TEST_F(SerialTest, test_calc_checksum_int32)
{
  // calc_checksum は static メソッドと仮定して Serial::calc_checksum で呼び出す
  // ケース1: ボディデータがすべてゼロ (64バイト) - これは共通
  std::vector<unsigned char> body1(PACKET_BODY_SIZE, 0);
  EXPECT_EQ(Serial::calc_checksum(body1.data(), body1.size()), 0xFFFF);

  // ケース2: 左エンコーダカウント = 1, 右 = 0
  std::vector<unsigned char> body2(PACKET_BODY_SIZE, 0);
  int32_t left_count1 = 1;   // 0x01 0x00 0x00 0x00 (リトルエンディアン)
  int32_t right_count1 = 0;
  memcpy(body2.data(), &left_count1, sizeof(int32_t));
  memcpy(body2.data() + sizeof(int32_t), &right_count1, sizeof(int32_t));
  // 期待値計算: 0x0001 + 0x0000 + 0x0000 + 0x0000 = 0x0001 => ~0x0001 = 0xFFFE
  EXPECT_EQ(Serial::calc_checksum(body2.data(), body2.size()), 0xFFFE);

  // ケース3: 左エンコーダカウント = -1, 右 = 0
  std::vector<unsigned char> body3(PACKET_BODY_SIZE, 0);
  int32_t left_count2 = -1;   // 0xFF 0xFF 0xFF 0xFF
  int32_t right_count2 = 0;
  memcpy(body3.data(), &left_count2, sizeof(int32_t));
  memcpy(body3.data() + sizeof(int32_t), &right_count2, sizeof(int32_t));
  // 期待値計算: 0xFFFF + 0xFFFF = 0x1FFFE => (0xFFFE + 1) = 0xFFFF => ~0xFFFF = 0x0000
  EXPECT_EQ(Serial::calc_checksum(body3.data(), body3.size()), 0x0000);

  // ケース4: 左エンコーダカウント = 1000, 右 = -2000
  std::vector<unsigned char> body4(PACKET_BODY_SIZE, 0);
  int32_t left_count3 = 1000;    // 0xE8 0x03 0x00 0x00
  int32_t right_count3 = -2000;   // 0x30 0xF8 0xFF 0xFF
  memcpy(body4.data(), &left_count3, sizeof(int32_t));
  memcpy(body4.data() + sizeof(int32_t), &right_count3, sizeof(int32_t));
  // 期待値計算: 0x03E8 + 0x0000 + 0xF830 + 0xFFFF = 0x1FC17 => (0xFC17 + 1) = 0xFC18 => ~0xFC18 = 0x03E7
  EXPECT_EQ(Serial::calc_checksum(body4.data(), body4.size()), 0x03E7);

  // ケース5: 左右に大きな値
  std::vector<unsigned char> body5(PACKET_BODY_SIZE, 0);
  int32_t left_count4 = 123456789;    // 0x15 CD 5B 07
  int32_t right_count4 = 987654321;   // 0xB1 E1 82 3A

  printf("Test Case 5 Data Setup:\n");
  printf("  left_count4 = %d (0x%08X)\n", left_count4, left_count4);
  printf("  right_count4 = %d (0x%08X)\n", right_count4, right_count4);

  memcpy(body5.data(), &left_count4, sizeof(int32_t));
  memcpy(body5.data() + sizeof(int32_t), &right_count4, sizeof(int32_t));
  // 期待値計算: 0xCD15 + 0x075B + 0xE1B1 + 0x3A82 = 0x1F0A3 => (0xF0A3 + 1) = 0xF0A4 => ~0xF0A4 = 0x0F5B
  printf("  body5 content after memcpy (first 8 bytes):\n  ");
  for (size_t k = 0; k < 8; ++k) {
    printf(" %02X", static_cast<unsigned char>(body5[k]));
  }
  printf("\n");
  EXPECT_EQ(Serial::calc_checksum(body5.data(), body5.size()), 0x87FF);
}


TEST_F(SerialTest, test_float_bin_roundtrip)
{
  // テストしたい float 値のリスト
  std::vector<float> test_values = {
    0.0f,
    1.0f,
    -1.0f,
    123.45f,
    -67.89f,
    std::numeric_limits<float>::max(),
    -std::numeric_limits<float>::max()
  };

  for (const auto & original_value : test_values) {
    // 1. float -> bin
    std::vector<unsigned char> bytes = Serial::float_to_bin(original_value);
    // 4バイトのベクトルが生成されることを確認
    EXPECT_EQ(bytes.size(), sizeof(float));

    // 2. bin -> float
    float roundtrip_value = Serial::bin_to_float(bytes.data());

    // 3. 元の値と、変換して戻してきた値が一致することを確認
    // EXPECT_FLOAT_EQ を使って浮動小数点数の誤差を許容して比較
    EXPECT_FLOAT_EQ(original_value, roundtrip_value);

    // デバッグ用に値を出力 (テストが失敗した場合に役立つ)
    if (original_value != roundtrip_value) {
      printf(
          "Mismatch for value %.8f: bin is [0x%02X, 0x%02X, 0x%02X, 0x%02X], roundtrip is %.8f\n",
          original_value, bytes[0], bytes[1], bytes[2], bytes[3], roundtrip_value);
    }
  }
}

TEST_F(SerialTest, test_int32_to_bin)
{
    // ケース1: 0
    int32_t val1 = 0;
    std::vector<unsigned char> expected1 = {0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(Serial::int32_to_bin(val1), expected1);

    // ケース2: 正の値
    int32_t val2 = 1000; // 16進数: 0x000003E8
    std::vector<unsigned char> expected2 = {0xE8, 0x03, 0x00, 0x00}; // リトルエンディアン
    EXPECT_EQ(Serial::int32_to_bin(val2), expected2);

    // ケース3: 負の値
    int32_t val3 = -2000; // 16進数: 0xFFFFF830
    std::vector<unsigned char> expected3 = {0x30, 0xF8, 0xFF, 0xFF}; // リトルエンディアン
    EXPECT_EQ(Serial::int32_to_bin(val3), expected3);

    // ケース4: 境界値 INT32_MAX
    int32_t val4 = std::numeric_limits<int32_t>::max(); // 0x7FFFFFFF
    std::vector<unsigned char> expected4 = {0xFF, 0xFF, 0xFF, 0x7F}; // リトルエンディアン
    EXPECT_EQ(Serial::int32_to_bin(val4), expected4);

    // ケース5: 境界値 INT32_MIN
    int32_t val5 = std::numeric_limits<int32_t>::min(); // 0x80000000
    std::vector<unsigned char> expected5 = {0x00, 0x00, 0x00, 0x80}; // リトルエンディアン
    EXPECT_EQ(Serial::int32_to_bin(val5), expected5);
}

TEST_F(SerialTest, test_bin_to_int32)
{
    // ケース1: 0
    unsigned char data1[] = {0x00, 0x00, 0x00, 0x00};
    int32_t expected1 = 0;
    EXPECT_EQ(Serial::bin_to_int32(data1), expected1);

    // ケース2: 正の値
    unsigned char data2[] = {0xE8, 0x03, 0x00, 0x00}; // 1000 (リトルエンディアン)
    int32_t expected2 = 1000;
    EXPECT_EQ(Serial::bin_to_int32(data2), expected2);

    // ケース3: 負の値
    unsigned char data3[] = {0x30, 0xF8, 0xFF, 0xFF}; // -2000 (リトルエンディアン)
    int32_t expected3 = -2000;
    EXPECT_EQ(Serial::bin_to_int32(data3), expected3);

    // ケース4: 境界値 INT32_MAX
    unsigned char data4[] = {0xFF, 0xFF, 0xFF, 0x7F}; // 0x7FFFFFFF (リトルエンディアン)
    int32_t expected4 = std::numeric_limits<int32_t>::max();
    EXPECT_EQ(Serial::bin_to_int32(data4), expected4);

    // ケース5: 境界値 INT32_MIN
    unsigned char data5[] = {0x00, 0x00, 0x00, 0x80}; // 0x80000000 (リトルエンディアン)
    int32_t expected5 = std::numeric_limits<int32_t>::min();
    EXPECT_EQ(Serial::bin_to_int32(data5), expected5);
}

TEST_F(SerialTest, test_packetserial_encode)
{
  // ケース1: 特殊文字を含まない単純なデータ
  std::vector<unsigned char> raw1 = {0x01, 0x02, 0x03};
  std::vector<unsigned char> expected1 = {0x01, 0x02, 0x03, 0xC0};
  EXPECT_EQ(Serial::encode(raw1), expected1);

  // ケース2: 区切り文字(0xC0)のみを含むデータ
  std::vector<unsigned char> raw2 = {0xC0};
  std::vector<unsigned char> expected2 = {0xDB, 0xDC, 0xC0}; // 0xC0 -> 0xDB, 0xDC にエスケープ
  EXPECT_EQ(Serial::encode(raw2), expected2);

  // ケース3: エスケープ文字(0xDB)のみを含むデータ
  std::vector<unsigned char> raw3 = {0xDB};
  std::vector<unsigned char> expected3 = {0xDB, 0xDD, 0xC0}; // 0xDB -> 0xDB, 0xDD にエスケープ
  EXPECT_EQ(Serial::encode(raw3), expected3);

  // ケース4: 両方の特殊文字を含むデータ
  std::vector<unsigned char> raw4 = {0x01, 0xC0, 0x02, 0xDB, 0x03};
  std::vector<unsigned char> expected4 = {0x01, 0xDB, 0xDC, 0x02, 0xDB, 0xDD, 0x03, 0xC0};
  EXPECT_EQ(Serial::encode(raw4), expected4);

  // ケース5: 空データ
  std::vector<unsigned char> raw5 = {};
  std::vector<unsigned char> expected5 = {0xC0}; // 区切り文字のみ
  EXPECT_EQ(Serial::encode(raw5), expected5);
}

TEST_F(SerialTest, test_packetserial_decode)
{
    // ケース1: 特殊文字を含まない単純なデータ
    std::vector<unsigned char> encoded1 = {0x01, 0x02, 0x03, 0xC0};
    std::vector<unsigned char> expected1 = {0x01, 0x02, 0x03};
    EXPECT_EQ(Serial::decode(encoded1), expected1);

    // ケース2: 区切り文字(0xC0)がエスケープされたデータ
    std::vector<unsigned char> encoded2 = {0xDB, 0xDC, 0xC0};
    std::vector<unsigned char> expected2 = {0xC0};
    EXPECT_EQ(Serial::decode(encoded2), expected2);

    // ケース3: エスケープ文字(0xDB)がエスケープされたデータ
    std::vector<unsigned char> encoded3 = {0xDB, 0xDD, 0xC0};
    std::vector<unsigned char> expected3 = {0xDB};
    EXPECT_EQ(Serial::decode(encoded3), expected3);

    // ケース4: 両方の特殊文字を含むデータ
    std::vector<unsigned char> encoded4 = {0x01, 0xDB, 0xDC, 0x02, 0xDB, 0xDD, 0x03, 0xC0};
    std::vector<unsigned char> expected4 = {0x01, 0xC0, 0x02, 0xDB, 0x03};
    EXPECT_EQ(Serial::decode(encoded4), expected4);

    // ケース5: 空データ (区切り文字のみ)
    std::vector<unsigned char> encoded5 = {0xC0};
    std::vector<unsigned char> expected5 = {};
    EXPECT_EQ(Serial::decode(encoded5), expected5);

    // --- 異常系のテスト ---

    // ケース6: 不正なエスケープシーケンス (0xDB の後に予期しないバイト)
    std::vector<unsigned char> invalid_data1 = {0xDB, 0xAA, 0xC0};
    // 不正な入力なので、例外がスローされることを期待する
    ASSERT_THROW(Serial::decode(invalid_data1), std::runtime_error);

    // ケース7: 末尾に区切り文字がない
    std::vector<unsigned char> invalid_data2 = {0x01, 0x02, 0x03};
    ASSERT_THROW(Serial::decode(invalid_data2), std::runtime_error);

    // ケース8: 空の入力 (区切り文字すらない)
    std::vector<unsigned char> invalid_data3 = {};
    ASSERT_THROW(Serial::decode(invalid_data3), std::runtime_error);
}

TEST_F(SerialTest, test_create_packet)
{
  // 1. 送信したいデータを用意
  cugo_ros2_control2::SendValue sv;
  sv.pc_port = 8888;
  sv.mcu_port = 8889;
  sv.l_rpm = 10.5f;
  sv.r_rpm = -20.25f;

  // 2. パケットを生成
  std::vector<unsigned char> packet = Serial::create_packet(sv);

  // 3. 生成されたパケットを検証

  // 3-1. サイズが72バイトであることを確認
  ASSERT_EQ(packet.size(), 72);

  // 3-2. ヘッダの各フィールドを確認
  // ポート番号 (リトルエンディアン)
  uint16_t src_port = *reinterpret_cast<uint16_t *>(&packet[0]);
  uint16_t dst_port = *reinterpret_cast<uint16_t *>(&packet[2]);
  EXPECT_EQ(src_port, sv.pc_port);
  EXPECT_EQ(dst_port, sv.mcu_port);

  // 長さ
  uint16_t length = *reinterpret_cast<uint16_t *>(&packet[4]);
  EXPECT_EQ(length, 72);

  // 3-3. ボディのデータを確認
  float l_rpm = *reinterpret_cast<float *>(&packet[8]);  // ボディの先頭から0バイト目
  float r_rpm = *reinterpret_cast<float *>(&packet[12]); // ボディの先頭から4バイト目
  EXPECT_FLOAT_EQ(l_rpm, sv.l_rpm);
  EXPECT_FLOAT_EQ(r_rpm, sv.r_rpm);

  // 3-4. チェックサムを確認
  uint16_t received_checksum = *reinterpret_cast<uint16_t *>(&packet[6]);
  // ボディ部分だけを抜き出して、チェックサムを再計算
  unsigned char * body_ptr = &packet[8];
  uint16_t calculated_checksum = Serial::calc_checksum(body_ptr, 64);
  EXPECT_EQ(received_checksum, calculated_checksum);
}
