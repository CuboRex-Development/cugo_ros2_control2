#include <gtest/gtest.h>
#include "cugo_ros2_control2/cugo.hpp"

using namespace cugo_ros2_control2;

// テストフィクスチャ
class CuGoTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // 必要に応じて初期化
  }

  void TearDown() override
  {
    // 必要に応じてクリーンアップ
  }

  // デフォルトコンストラクタで初期化された CuGo インスタンス
  CuGo cugo_default;

  // パラメータ付きコンストラクタで初期化された CuGo インスタンス
  CuGo cugo_custom{0.044f, 0.045f, 0.8f};
};

// 静的環境のテスト
TEST_F(CuGoTest, test_static)
{
  // 初期化パラメータなど固定なものの代入が正しいか
  ASSERT_EQ(cugo_default.get_tread(), 0.376f);
  ASSERT_EQ(cugo_default.get_l_wheel_radius(), 0.03858f);
  ASSERT_EQ(cugo_default.get_r_wheel_radius(), 0.03858f);
  ASSERT_EQ(cugo_custom.get_tread(), 0.8f);
  ASSERT_EQ(cugo_custom.get_l_wheel_radius(), 0.044f);
  ASSERT_EQ(cugo_custom.get_r_wheel_radius(), 0.045f);
}

// パラメータ付きコンストラクタで初期化されたインスタンスの calc_rpm メソッドをテスト
TEST_F(CuGoTest, CalcRPM_CustomConstructor)
{
  // テスト入力
  //float linear_x = 2.0f;   // m/s
  //float angular_z = 1.0f;  // rad/s
  /*
  // calc_rpm メソッドの呼び出し
  RPM rpm = cugo_default.calc_rpm(linear_x, angular_z);

  // 期待される RPM（手動計算された値）
  float expected_l_rpm = 432.91f;
  float expected_r_rpm = 522.43f;

  // 結果の検証
  EXPECT_NEAR(rpm.l_rpm, expected_l_rpm, 1e-2);
  EXPECT_NEAR(rpm.r_rpm, expected_r_rpm, 1e-2);
   */
}
