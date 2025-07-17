![image](https://github.com/user-attachments/assets/be603edd-43dd-42b7-8215-2a89df03e3c2)

# cugo_ros2_control2

クローラロボット開発プラットフォームのROS 2ノードです。

ROS 2 topicの`/cmd_vel`をSubscribeし、`/odom`をPublishします。
セットでArduinoスケッチの[cugo_ros2_motorcontroller2](https://github.com/CuboRex-Development/cugo_ros2_motorcontroller2)と使用します。

ROS 2 Humble以降でご利用いただけます。

# Table of Contents
- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Topics and Parameters](#topics-and-parameters)
- [Note](#note)
- [License](#license)

# Features
Subscribeした`/cmd_vel`の速度ベクトルになるような仮想車輪L/Rの回転数を計算します。

計算した回転数をロボットのマイコンに送信します。

また、[cugo_ros2_motorcontroller2](https://github.com/CuboRex-Development/cugo_ros2_motorcontroller2)が書き込まれたロボットのマイコンからエンコーダのカウント数を受け取ります。

カウント数からロボットのオドメトリを計算し、`/odom`としてPublishします。

<img width="2527" height="1116" alt="image" src="https://github.com/user-attachments/assets/a8950d77-9907-4d95-99be-b6ca8f536b85" />



#### 対応製品

* [クローラロボット開発プラットフォーム CuGo V4](https://cuborex.com/product/?id=9)
* クローラロボット開発プラットフォーム CuGo V3i（現在販売終了）

でお使いいただけます。


# Requirements
- OS: Ubuntu 22.04.4 LTS / ROS Distribution: ROS 2 Humble Hawksbill
- OS: Ubuntu 24.04.4 LTS / ROS Distribution: ROS 2 Jazzy Jalisco
- xacro
- robot_state_publisher


# Installation
ROS 2環境がない場合は[ROS 2 Documentation](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)を参照しROS 2をインストールしてください。


ROS 2のワークスペース内でgit cloneしたのち、colcon buildしてください。
~~~
$ cd ~/your_ros2_ws/src
$ git clone https://github.com/CuboRex-Development/cugo_ros2_control2.git
$ cd ../..
$ colcon build --symlink-install
$ source ~/your_ros2_ws/install/local_setup.bash
~~~

ビルドエラーが発生する場合、依存パッケージをインストールしてから再度`colcon build`してください。
~~~
$ rosdep install -i --from-paths ~/your_ros2_ws/src/cugo_ros2_control2
$ cd ~/your_ros2_ws
$ colcon build --symlink-install
$ source ~/your_ros2_ws/install/local_setup.bash
~~~

# Usage

下記のコマンドでcugo_ros2_control2ノードが起動します。お手持ちのCuGoV3i / CuGoV4にあったlaunchファイルを指定してください。最適なパラメータが使用されます。

launchファイルのパラメータを変更することで微調整することもできます。詳細は[Parameters](#parameters)の項目を参照してください。

#### クローラロボット開発プラットフォーム CuGo V4の方
付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。

~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 777 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control2 cugov4_ros2_control_launch.py
~~~

#### クローラロボット開発プラットフォーム CuGo V3iの方

付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。
~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 777 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control2 cugov3i_ros2_control_launch.py
~~~

#### ロボット側の操作
クローラロボット開発プラットフォーム付属のRaspberryPiPicoに[こちらのスケッチ](https://github.com/CuboRex-Development/cugo_ros2_motorcontroller2)を書き込み、ROS 2 PCとRaspberryPiPicoをUSBケーブルで接続してください。
その後ROSパッケージを実行してください。自動で通信開始します。

スケッチの書き換えはROS PCである必要性はありません。

もし、うまく `/cmd_vel` 通りに走行を開始しない場合は、一度USBケーブルを抜き、
ロボットの電源を入れなおしてから再度PCとRaspberryPiPicoをUSBケーブルで接続してください。



# Topics and Parameters
## Published Topics
- `/odom` ([nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
- `/tf` ([tf2_msgs/msg/TFMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html))

## Subscription Topic
- `/cmd_vel` ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))

## Parameters
- `odom_frame_id (string, default: odom)`
  - オドメトリフレーム名の指定
- `base_link_frame_id (string, default: base_link)`
  - ベースリンクフレーム名の指定
- `subscribe_topic_name (string, default: /cmd_vel)`
  - Twist指示のトピック名の指定
- `publish_topic_name (string, default: /odom)`
  - Odometry出力のトピック名の指定
- `control_frequency (float, default: 10.0)`
  - マイコンへの指示、OdomのPublishの更新周期（最大100Hz）
- `serial_port (string, default: /dev/ttyACM0)`
  - RaspberryPi Picoのシリアル通信のポート名の指定
- `serial_baudrate (int, default: 115200)`
  - シリアル通信のボーレート
- `cmd_vel_timeout (float, default: 0.5)`
  - /cmd_velの通信途絶判定を決めるタイムアウト時間[秒]
  - タイムアウトしたら速度0を上書きして強制的に停止
- `serial_timeout (float, default: 0.5)`
  - マイコンの通信途絶判定を決めるタイムアウト時間[秒]
  - タイムアウトしたらodom.twsitの値を0にして仮想ロボット速度をリセット
- `tread (float, default: 0.376)`
  - 回転ベクトルを計算するときに使用するクローラ進行方向に対する左右方向の距離[m]
  - アルミフレームでクローラ間距離を変えた場合この値を調整
- `l_wheel_radius (float, default: 0.03858)`
  - 左クローラの仮想タイヤ半径[m]
  - まっすぐ走らせてオドメトリがだんだん左に曲がっていく場合この値を少しだけ大きくするとよい（0.00002m刻み）
- `r_wheel_radius (float, default: 0.03858)`
  - 右クローラの仮想タイヤ半径[m]
  - まっすぐ走らせてオドメトリがだんだん右に曲がっていく場合この値を少しだけ大きくするとよい（0.00002m刻み）
- `reduction_ratio (float, default: 20.0)`
  - ロボットに搭載されるギヤボックスの減速比
  - ギヤボックスを変更した場合この値を調整
- `encoder_resolution (int, default: 30)`
  - モータ1回転あたりのエンコーダカウント数
  - モータを交換した場合この値を調整
- `pose_cov_x (float, default: 0.025)`
  - Odometryの`pose.x`の共分散
  - 0.05[m]程度の精度を見込んでいる(0.05^2)
- `pose_cov_y (float, default: 0.025)`
  - Odometryの`pose.y`の共分散
  - 0.05[m]程度の精度を見込んでいる(0.05^2)
- `pose_cov_z (float, default: 1e9)`
  - Odometryの`pose.z`の共分散
  - 2次元平面のみの定義であるため、無限大に設定している
- `pose_cov_roll (float, default: 1e9)`
  - Odometryの`orientation.q`のroll成分の共分散
  - 2次元平面のみの定義であるため、無限大に設定している
- `pose_cov_pitch (float, default: 1e9)`
  - Odometryの`orientation.q`のpitch成分の共分散
  - 2次元平面のみの定義であるため、無限大に設定している
- `pose_cov_yaw (float, default: 0.0001)`
  - Odometryの`orientation.q`のyaw成分の共分散
  - 0.01[rad]程度の精度を見込んでいる(0.01^2)
- `twist_cov_linear_x (float, default: 0.0001)`
  - Twistの`linear.x`の共分散
  - 0.01[m/s]程度の精度を見込んでいる(0.01^2)
- `twist_cov_angular_z (float, default: 0.0001)`
  - Twistの`angular.z`の共分散
  - 0.01[rad/s]程度の精度を見込んでいる(0.01^2)

上記のパラメータはlaunchファイルで設定されています。

# TF
CuGoを活用したロボットでTFを構築するためにxacroを利用します。

ご自身のロボットに取り付けられている部品を説明するxacroを`/urdf/parts`ディレクトリに格納してください。
デフォルトでは、
- CuGoそのものの位置関係を表現したurdfの`cugo_base.urdf.xacro`
- 部品を追加したサンプルとしてのurdfの`mid360.urdf.xacro` (MID-360は製品には付属していません。コメントアウトで無効化されています)
が格納されています。
~~~
cugo_ros2_control2
└── urdf
    ├── my_cugo_robot.urdf.xacro
    └── parts
        ├── cugo_base.urdf.xacro
        └── mid360.urdf.xacro
~~~

`parts`内にある部品xacroを`my_cugo_robot.urdf.xacro`が読み込むことでロボット全体のTFを構築することができます。
`my_cugo_robot.urdf.xacro` は `robot_state_publisher` によって、ロボット構成のTFを配信します。
部品を追加する場合、`my_cugo_robot.urdf.xacro`にご自身で追加したxacro名を追記してください。

追記した後は`colcon build`を行ってください。追加したファイルが反映されます。

# Protocol
[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)と、送信・受信ともにヘッダ8バイト・ボディ64バイトの合計72バイトから構成されるデータを通信しています。
ボディデータに格納されるデータの一覧は以下の通りになります。
ボディの残りの領域は今後拡張できるように確保されているだけで、現在は00を送受信しています。

### Arduinoドライバへの送信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Description
---------------|------------|-----------------|-----------------------------|--------------------
TARGET_RPM_L   | float      | 4               | 0                           | RPM指令値(左モータ)
TARGET_RPM_R   | float      | 4               | 4                           | RPM指令値(右モータ)


### Arduinoドライバからの受信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Description
---------------|------------|-----------------|-----------------------------|-----------------
RECV_ENCODER_L | int32      | 4               | 0                           | 左エンコーダのカウント数
RECV_ENCODER_R | int32      | 4               | 4                           | 右エンコーダのカウント数


# Note

ご不明点がございましたら、[お問い合わせフォーム](https://cuborex.com/contact/)にてお問い合わせください。回答いたします。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
