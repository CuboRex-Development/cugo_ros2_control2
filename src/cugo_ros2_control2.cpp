#include "cugo_ros2_control2/cugo_ros2_control2.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cugo_ros2_control2");
    RCLCPP_INFO(node->get_logger(), "Cugo ROS 2 Control Node has started.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
