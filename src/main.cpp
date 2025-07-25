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

#include "cugo_ros2_control2/node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cugo_ros2_control2::Node>();
  rclcpp::executors::MultiThreadedExecutor executor; // マルチスレッドエグゼキュータ
  executor.add_node(node);
  RCLCPP_INFO(node->get_logger(), "Cugo ROS 2 Control Node has started.");
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
