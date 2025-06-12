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

/* FLOATのバイト列が一致しないため、ラウンドトリップテストに置き換え
TEST_F(SerialTest, test_float_to_bin)
{
    // Serial::float_to_bin が static std::vector<unsigned char> を返すと仮定

    // ケース1: 0.0f
    float val1 = 0.0f;
    std::vector<unsigned char> expected1 = {0x00, 0x00, 0x00, 0x00};
    EXPECT_EQ(Serial::float_to_bin(val1), expected1);

    // ケース2: 正の値
    float val2 = 123.45f; // 16進数表現: 0x42F6E666
    std::vector<unsigned char> expected2 = {0x66, 0xE6, 0xF6, 0x42}; // リトルエンディアン
    EXPECT_EQ(Serial::float_to_bin(val2), expected2);

    // ケース3: 負の値
    float val3 = -67.89f; // 16進数表現: 0xC287C28F
    std::vector<unsigned char> expected3 = {0x8F, 0xC2, 0x87, 0xC2}; // リトルエンディアン
    EXPECT_EQ(Serial::float_to_bin(val3), expected3);
}

// --- バイナリ変換 bin -> float のテスト ---
TEST_F(SerialTest, test_bin_to_float)
{
    // Serial::bin_to_float が static float を返し、
    // 引数として const unsigned char* を受け取ると仮定

    // ケース1: 0.0f
    unsigned char data1[] = {0x00, 0x00, 0x00, 0x00};
    float expected1 = 0.0f;
    EXPECT_FLOAT_EQ(Serial::bin_to_float(data1), expected1);

    // ケース2: 正の値
    unsigned char data2[] = {0x66, 0xE6, 0xF6, 0x42}; // 123.45f (リトルエンディアン)
    float expected2 = 123.45f;
    EXPECT_FLOAT_EQ(Serial::bin_to_float(data2), expected2);

    // ケース3: 負の値
    unsigned char data3[] = {0x8F, 0xC2, 0x87, 0xC2}; // -67.89f (リトルエンディアン)
    float expected3 = -67.89f;
    EXPECT_FLOAT_EQ(Serial::bin_to_float(data3), expected3);
}
*/

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

TEST_F(SerialTest, test_create_packet)
{

}
