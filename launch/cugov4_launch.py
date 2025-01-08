from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # パラメータの定義
    parameters = {
        'control_frequency': 10.0,
        # 'diagnostic_frequency': 1.0,
        'serial_port': '/dev/ttyACM0',
        'serial_baudrate': 115200,
        'cmd_vel_timeout': 0.5,   # 秒
        'serial_timeout': 0.5     # 秒
        # 必要に応じて他のパラメータも追加
    }

    return LaunchDescription([
        Node(
            package='cugo_ros2_control2',
            executable='cugo_ros2_control2',
            name='cugo_ros2_control2',
            output='screen',
            parameters=[parameters],
            emulate_tty=True
        )
    ])
