#include <gtest/gtest.h>
#include "cugo_ros2_control2/cugo.hpp"

/*
// 既存のテストコード
TEST(CuGoTest, InitializationTest)
{
  EXPECT_EQ(0, 0);
}

// 失敗するテストコード
TEST(CuGoTest, FailingTest)
{
  // 意図的に失敗させる条件
  EXPECT_EQ(0, 1);
}
*/

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
