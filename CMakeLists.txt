cmake_minimum_required(VERSION 3.8)
project(cugo_ros2_control2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依存パッケージの検索
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)

# 実行ファイルの作成
add_executable(cugo_ros2_control2 src/node.cpp src/cugo.cpp src/serial.cpp)
#add_library(cugo src/cugo.cpp)
#add_library(serial src/serial.cpp)
#target_link_libraries(cugo_ros2_control2 cugo serial)

# インクルードディレクトリの追加
include_directories(
  include
)

ament_target_dependencies(
  cugo_ros2_control2
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
  diagnostic_updater
)

target_compile_features(cugo_ros2_control2 PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS cugo_ros2_control2
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_node test/node.cpp)
  #target_link_libraries(node_test cugo_ros2_control2)
endif()

ament_package()
