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
  CuGo cugo_custom{0.044f, 0.045f, 0.8f, 10.0, 160};
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
  ASSERT_EQ(cugo_custom.get_reduction_ratio(), 10.0f);
  ASSERT_EQ(cugo_custom.get_encoder_resolution(), 160);
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

// エンコーダカウントから計算するtwistが正しいか
TEST_F(CuGoTest, test_calc_twist)
{
  Twist twist;
  twist = cugo_default.calc_twist(0, 0, 0.1);
  ASSERT_NEAR(twist.linear_x, 0.0, 1e-4);
  ASSERT_NEAR(twist.angular_z, 0.0, 1e-4);

  twist = cugo_default.calc_twist(100, 100, 0.1);
  ASSERT_NEAR(twist.linear_x, 0.03366740127, 1e-4);
  ASSERT_NEAR(twist.angular_z, 0.0, 1e-4);

  twist = cugo_default.calc_twist(100, -100, 0.1);
  ASSERT_NEAR(twist.linear_x, 0.0, 1e-4);
  ASSERT_NEAR(twist.angular_z, -0.1790819217, 1e-4);

  twist = cugo_default.calc_twist(200, 100, 0.1);
  ASSERT_NEAR(twist.linear_x, 0.0505011019, 1e-4);
  ASSERT_NEAR(twist.angular_z, -0.08954096082, 1e-4);
}

// twistの累積が正しいかどうか
TEST_F(CuGoTest, test_calc_odom)
{
  Odom odom;
  odom.x = 0.0;
  odom.y = 0.0;
  odom.yaw = 0.0;
  Twist twist;
  twist.linear_x = 0.0;
  twist.angular_z = 0.0;
  float dt = 0.1;

  odom = cugo_default.calc_odom(odom, twist, dt);
  ASSERT_NEAR(odom.x, 0.0, 1e-4);
  ASSERT_NEAR(odom.y, 0.0, 1e-4);
  ASSERT_NEAR(odom.yaw, 0.0, 1e-4);

  twist.linear_x = 0.5;
  twist.angular_z = 0.0;
  odom = cugo_default.calc_odom(odom, twist, dt);
  ASSERT_NEAR(odom.x, 0.05, 1e-4);
  ASSERT_NEAR(odom.y, 0.0, 1e-4);
  ASSERT_NEAR(odom.yaw, 0.0, 1e-4);

  twist.linear_x = 0.3;
  twist.angular_z = 1.57;
  odom = cugo_default.calc_odom(odom, twist, dt);
  ASSERT_NEAR(odom.x, 0.07963102384, 1e-4);
  ASSERT_NEAR(odom.y, 0.004690674368, 1e-4);
  ASSERT_NEAR(odom.yaw, 0.157, 1e-4);

  twist.linear_x = 0.5;
  twist.angular_z = -3.14;
  odom = cugo_default.calc_odom(odom, twist, dt);
  ASSERT_NEAR(odom.x, 0.1290160636, 1e-4);
  ASSERT_NEAR(odom.y, -0.003127116246, 1e-4);
  ASSERT_NEAR(odom.yaw, -0.157, 1e-4);
}
