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
