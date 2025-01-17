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

using namespace cugo_ros2_control2;

class NodeTest : public ::testing::Test
{
protected:
  // テストスイート全体で一度だけ実行されるセットアップ
  static void SetUpTestSuite()
  {
    // ROS 2 の初期化
    rclcpp::init(0, nullptr);
  }

  // テストスイート全体で一度だけ実行されるティアダウン
  static void TearDownTestSuite()
  {
    // ROS 2 のシャットダウン
    rclcpp::shutdown();
  }

  // 各テストケース前に実行されるセットアップ
  void SetUp() override
  {
    // Nodeのインスタンス化
    node = std::make_shared<Node>();
  }

  // 各テストケース後に実行されるティアダウン
  void TearDown() override
  {
    // Nodeのクリーンアップ
    node.reset();
  }

  std::shared_ptr<Node> node;
};

// cmdvelのタイムアウト算出
TEST_F(NodeTest, test_check_dt)
{
  double prevtime = 1.0;
  double recvtime = 2.0;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), 1.0, 1e-2);

  prevtime = 1.0;
  recvtime = 1.0;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), 0.0, 1e-2);

  prevtime = 1.0;
  recvtime = 0.0;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), -1.0, 1e-2);
}

// タイムアウト検出が正しいかどうか
TEST_F(NodeTest, test_is_timeout)
{
  double prevtime = 0.0;
  double recvtime = 0.0;
  double timeout = 0.5;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), false);

  prevtime = 100.0;
  recvtime = 101.0;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), true);

  // イコールはタイムアウト
  prevtime = 100.0;
  recvtime = 100.5;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), true);

  prevtime = 100.0;
  recvtime = 101.0;
  timeout = 2.0;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), false);
}

TEST_F(NodeTest, test_set_zero_rpm)
{
  RPM rpm = node->set_zero_rpm();
  ASSERT_NEAR(rpm.l_rpm, 0.0, 1e-4);
  ASSERT_NEAR(rpm.r_rpm, 0.0, 1e-4);
}

TEST_F(NodeTest, test_is_sametime)
{
  double prevtime = 0.0;
  double recvtime = 0.0;
  ASSERT_EQ(node->is_sametime(recvtime, prevtime), true);

  prevtime = 0.0;
  recvtime = 0.1;
  ASSERT_EQ(node->is_sametime(recvtime, prevtime), false);

  prevtime = 66666666.6;
  // 時間更新されていなかったら同じ変数をコピーしてくる
  recvtime = prevtime;
  ASSERT_EQ(node->is_sametime(recvtime, prevtime), true);

  prevtime = 66666666.6;
  recvtime = 66666667.8;
  ASSERT_EQ(node->is_sametime(recvtime, prevtime), false);
}

TEST_F(NodeTest, test_is_illegaltime)
{
  double prevtime = 0.0;
  double recvtime = 0.0;
  ASSERT_EQ(node->is_illegaltime(recvtime, prevtime), false);

  prevtime = 0.0;
  recvtime = 0.1;
  ASSERT_EQ(node->is_illegaltime(recvtime, prevtime), false);

  prevtime = 10.0;
  recvtime = 9.9;
  ASSERT_EQ(node->is_illegaltime(recvtime, prevtime), true);

  prevtime = 66666666.6;
  recvtime = 66666667.8;
  ASSERT_EQ(node->is_illegaltime(recvtime, prevtime), false);

  prevtime = 66666667.8;
  recvtime = 66666666.6;
  ASSERT_EQ(node->is_illegaltime(recvtime, prevtime), true);
}
