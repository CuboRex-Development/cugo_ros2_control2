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

// 初期化パラメータなど固定なものの代入が正しいか
TEST_F(CuGoTest, test_initialize)
{
  ASSERT_EQ(cugo_default.get_tread(), 0.376f);
  ASSERT_EQ(cugo_default.get_l_wheel_radius(), 0.03858f);
  ASSERT_EQ(cugo_default.get_r_wheel_radius(), 0.03858f);
  ASSERT_EQ(cugo_custom.get_tread(), 0.8f);
  ASSERT_EQ(cugo_custom.get_l_wheel_radius(), 0.044f);
  ASSERT_EQ(cugo_custom.get_r_wheel_radius(), 0.045f);
}

// calc_rpm()の出力値テスト
TEST_F(CuGoTest, test_calc_rpm)
{
  // 速度0入力
  float linear_x = 0.0f;
  float angular_z = 0.0f;
  RPM rpm = cugo_default.calc_rpm(linear_x, angular_z);
  ASSERT_EQ(rpm.l_rpm, 0.0f);
  ASSERT_EQ(rpm.r_rpm, 0.0f);

  // 前後進のみ入力
  linear_x = 0.5f;
  angular_z = 0.0f;
  rpm = cugo_default.calc_rpm(linear_x, angular_z);
  ASSERT_NEAR(rpm.l_rpm, 123.7596758f, 1e-2);
  ASSERT_NEAR(rpm.r_rpm, 123.7596758f, 1e-2);

  // 回転のみ入力
  linear_x = -0.5f;
  angular_z = 0.0f;
  rpm = cugo_default.calc_rpm(linear_x, angular_z);
  ASSERT_NEAR(rpm.l_rpm, -123.7596758f, 1e-2);
  ASSERT_NEAR(rpm.r_rpm, -123.7596758f, 1e-2);

  // 曲がりながら走行するベクトルを入力
  linear_x = 0.5f;
  angular_z = 1.0f;
  rpm = cugo_default.calc_rpm(linear_x, angular_z);
  ASSERT_NEAR(rpm.l_rpm, 77.22603771f, 1e-2);
  ASSERT_NEAR(rpm.r_rpm, 170.2933139f, 1e-2);
}
