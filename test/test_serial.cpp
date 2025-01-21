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
#include "cugo_ros2_control2/node.hpp"
#include "cugo_ros2_control2/serial.hpp"

using namespace cugo_ros2_control2;

// テストフィクスチャ
class CuGoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // テスト用のSerialインスタンスを初期化
    test_serial = std::make_shared<Serial>("/dev/ttyACM0", 115200);
  }

  void TearDown() override
  {
    // クリーンアップ
    if (test_serial && test_serial->serial_port_.is_open()) {
      test_serial->close();
    }
  }

  std::shared_ptr<Serial> test_serial;
  //std::shared_ptr<Serial> open_failed_serial;
};

// USBとRaspberryPiPicoを接続してcolcon build / colcon testすること
TEST_F(CuGoTest, test_open)
{
  // シリアルポートを開くテスト
  EXPECT_NO_THROW(
  {
    test_serial->open("/dev/ttyACM0", 115200);
  });
  EXPECT_TRUE(test_serial->serial_port_.is_open());

  // 存在しないポートを開こうとすると例外を投げるテスト
  // TODO:一回クローズして存在しないポートで開く
  /*
  EXPECT_THROW(
  {
    test_serial->open("/dev/null", 115200);
  }, boost::system::system_error);
  */
}
