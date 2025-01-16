#include <gtest/gtest.h>
#include "cugo_ros2_control2/node.hpp"

using namespace cugo_ros2_control2;

class NodeTest : public ::testing::Test
{
protected:
  void SetUp() override
  {

  }

  void TearDown() override
  {

  }

//  Node node;
};

// cmdvelのタイムアウト算出
TEST_F(NodeTest, test_check_dt)
{
  /*
  rclcpp::Time prevtime = node.get_clock()->now();
  rclcpp::Time recvtime = node.get_clock()->now();
  ASSERT_NEAR(node.check_difftime(recvtime, prevtime), 0.0, 1e-2);
  */
}
