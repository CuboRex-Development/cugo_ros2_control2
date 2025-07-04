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
        'tread': 0.376,             # cugov4のトレッド幅
        'l_wheel_radius': 0.03858,  # cugov4のスプロケット半径
        'r_wheel_radius': 0.03858,  # cugov4のスプロケット半径
        'reduction_ratio': 20.0,    # cugov4のオリエンタルモータの減速比
        'encoder_resolution': 30,   # cugov4のオリエンタルモータのエンコーダホール数
        'product_id': 1,            # cugov4の識別子

        # 共分散の設定
        # SLAMやLocalizationで調整
        # オドメトリで悪影響がある場合は調整してください
        # オドメトリの自己位置
        'pose_cov_x': 0.025, # 0.05m^2
        'pose_cov_y': 0.025, # 0.05m^2
        'pose_cov_z': 1e9,
        'pose_cov_roll': 1e9,
        'pose_cov_pitch': 1e9,
        'pose_cov_yaw': 0.01, # 0.1rad^2

        # ロボットの速度
        'twist_cov_linear_x': 0.0001, # 0.01m^2
        'twist_cov_angular_z': 0.0001, # 0.01m^2
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
