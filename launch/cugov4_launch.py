from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # ログレベルの変更
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='debug',
        description='Log level: debug, info, warn, error, fatal'
    )
    log_level = LaunchConfiguration('log_level')

    # パラメータの定義
    parameters = {
        'control_frequency': 10.0,  # MAX:100.0
        # 'diagnostic_frequency': 1.0,
        'serial_port': '/dev/ttyACM0',
        'serial_baudrate': 115200,
        'cmd_vel_timeout': 0.5,   # 秒
        'serial_timeout': 0.5     # 秒
        # 必要に応じて他のパラメータも追加
    }

    # ノードの定義
    cugo_node = Node(
        package='cugo_ros2_control2',
        executable='cugo_ros2_control2',
        name='cugo_ros2_control2',
        output='screen',
        parameters=[parameters],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level]
    )

    return LaunchDescription([
        log_level_arg,  # DeclareLaunchArgumentをLaunchDescriptionに含める
        cugo_node
    ])
