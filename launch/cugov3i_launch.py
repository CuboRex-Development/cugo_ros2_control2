from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    # ログレベルの変更
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level: debug, info, warn, error, fatal'
    )
    log_level = LaunchConfiguration('log_level')

    # ---- robot_state_publisher の設定 ----
    # パッケージの共有ディレクトリのパスを取得
    pkg_share = get_package_share_directory('cugo_ros2_control2')

    # トップレベルのxacroファイルへのパス
    xacro_file = os.path.join(pkg_share, 'urdf', 'my_cugo_robot.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = {'robot_description': doc.toxml()}

    # robot_state_publisherノードの定義
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_config]
    )

    # ---- cugo_ros2_control2 の設定 ----
    # パラメータの定義
    parameters = {
        'odom_frame_id': 'odom',
        'base_link_frame_id': 'base_footprint',
        'subscribe_topic_name': '/cmd_vel',
        'publish_topic_name': '/odom',
        'control_frequency': 10.0,  # MAX:100.0
        'serial_port': '/dev/ttyACM0',
        'serial_baudrate': 115200,
        'cmd_vel_timeout': 0.5,     # 秒
        'serial_timeout': 0.5,      # 秒
        'tread': 0.380,             # cugov3iのトレッド幅
        'l_wheel_radius': 0.03858,  # cugov3iのスプロケット半径
        'r_wheel_radius': 0.03858,  # cugov3iのスプロケット半径
        'reduction_ratio': 15.0,    # cugov3iのオリエンタルモータの減速比
        'encoder_resolution': 24,   # cugov3iのオリエンタルモータのエンコーダホール数
        'product_id': 1             # cugov3iの識別子
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
        cugo_node,
        robot_state_publisher_node
    ])
