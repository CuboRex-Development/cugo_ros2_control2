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
  float prevtime = 1.0f;
  float recvtime = 2.0f;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), 1.0, 1e-2);

  prevtime = 1.0f;
  recvtime = 1.0f;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), 0.0, 1e-2);

  prevtime = 1.0f;
  recvtime = 0.0f;
  ASSERT_NEAR(node->check_difftime(recvtime, prevtime), -1.0, 1e-2);
}

// タイムアウト検出が正しいかどうか
TEST_F(NodeTest, test_is_timeout)
{
  float prevtime = 0.0f;
  float recvtime = 0.0f;
  float timeout = 0.5f;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), false);

  prevtime = 100.0f;
  recvtime = 101.0f;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), true);

  prevtime = 100.0f;
  recvtime = 100.5f;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), true);

  prevtime = 100.0f;
  recvtime = 101.0f;
  timeout = 2.0f;
  ASSERT_EQ(node->is_timeout(recvtime, prevtime, timeout), false);

}
