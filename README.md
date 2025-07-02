# cugo_ros2_control2
![image](https://github.com/user-attachments/assets/be603edd-43dd-42b7-8215-2a89df03e3c2)


クローラロボット開発プラットフォームのROS 2ノードです。

ROS 2 topicの`/cmd_vel`をSubscribeし、`/odom`をPublishします。
セットで[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_arduinodriver.git)使用します。

ROS 2 Humble以降でご利用いただけます。

# Table of Contents
- [Features](#features)
- [Requirement](#requirement)
- [Installation](#installation)
- [Usage](#usage)
- [Topics and Parameters](#topics-and-parameters)
- [Note](#note)
- [License](#license)

# Features
Subscribeした`/cmd_vel`の速度ベクトルになるような仮想車輪L/Rの回転数を計算します。

計算した回転数をロボットのマイコンに送信します。

また、[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)が書き込まれたロボットのマイコンからエンコーダのカウント数を受け取ります。

カウント数からロボットのオドメトリを計算し、`/odom`を生成しPublishします。

![image](https://github.com/user-attachments/assets/f12eef02-0bb9-4654-890b-ffc72c7c708e)


#### 対応製品
CuboRex製品では、
* クローラロボット開発プラットフォーム CuGo V4
* クローラロボット開発プラットフォーム CuGo V3i

でお使いいただけます。


クローラロボット開発プラットフォーム付属のRaspberryPiPicoに[こちらのスケッチ](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)を書き込み、ROS 2 PCとRaspberryPiPicoをUSBケーブルで接続してください。
その後ROSパッケージを実行してください。自動で通信開始します。


スケッチの書き換えはROS PCである必要性はありません。


# Requirement
- OS: Ubuntu 22.04.4 LTS / ROS Distribution: ROS 2 Humble Hawksbill
- OS: Ubuntu 24.04.4 LTS / ROS Distribution: ROS 2 Jazzy Jalisco
- xacro


# Installation
ROS 2環境がない場合は[ROS 2 Documentation](https://docs.ros.org/en/Jazzy/Installation/Ubuntu-Install-Debians.html)を参照しROS 2をインストールしてください。

xacroパッケージを使用しているため、aptでインストールしてください。
~~~
$ sudo apt install ros-$ROS_DISTRO-xacro
~~~

ROS 2のワークスペース内でgit cloneしたのち、colcon buildしてください。
~~~
$ cd ~/your/ros_workspace/ros2_ws/src
$ git clone https://github.com/CuboRex-Development/cugo_ros2_control2.git
$ cd ../..
$ colcon build --symlink-install
$ source ~/your/ros_workspace/ros2_ws/install/local_setup.bash
~~~

# Usage

下記のコマンドでcugo_ros2_control2ノードが起動します。お手持ちのCuGoV3i / CuGoV4にあったlaunchファイルを指定してください。最適なパラメータが使用されます。launchファイルのパラメータを変更することで微調整することもできます。詳細は[Parameters](#parameters)の項目を参照してください。

#### クローラロボット開発プラットフォーム CuGo V4の方
付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。

~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 777 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control cugov4_ros2_control_launch.py
~~~

#### クローラロボット開発プラットフォーム CuGo V3iの方

付属のRaspberryPiPicoとUSBケーブルで接続をしたのち、お客様環境にあった権限設定をしてからlaunchファイルを実行してください。
~~~
# RaspberryPiPicoの権限付与例
# お客様環境に合わせてコマンドを実行してください。
$ sudo chmod 777 /dev/ttyACM0

# launch ファイルを実行
$ ros2 launch cugo_ros2_control cugov3i_ros2_control_launch.py
~~~

# Topics and Parameters
## Published Topics
- `/odom` ([nav_msgs/msg/Odometry](https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html))
- `/tf` ([tf2_msgs/msg/TFMessage](https://docs.ros2.org/foxy/api/tf2_msgs/msg/TFMessage.html))

## Subscribed Topics
- `/cmd_vel` ([geometry_msgs/msg/Twist](https://docs.ros2.org/foxy/api/geometry_msgs/msg/Twist.html))

## Parameters
- `odom_frame_id (string, default: odom)`
  - オドメトリフレーム名の指定
- `base_link_frame_id (string, default: base_link)`
  - ベースリンクフレーム名の指定
- `subscribe_topic_name (string, default: /cmd_vel)`
  - Twist指示のトピック名の指定
- `publish_topic_name (string, default: /odom)`
  - Odom出力のトピック名の指定
- `control_frequency (float, default: 10.0)`
  - マイコンへの指示、OdomのPublishの更新周期
- `serial_port (string, default: /dev/ttyACM0)`
  - RaspberryPi Picoのシリアル通信のポート名の指定
- `serial_baudrate (int, default: 115200)`
  - シリアル通信のボーレート
- `cmd_vel_timeout (float, default: 0.5)`
  - /cmd_velの通信途絶判定を決めるタイムアウト時間
  - タイムアウトしたら速度0を上書きして強制的に停止
- `serial_timeout (float, default: 0.5)`
  - マイコンの通信途絶判定を決めるタイムアウト時間
  - タイムアウトしたらodom.twsitの値を0にして仮想ロボット速度をリセット
- `tread (float, default: 0.376)`
  - 回転ベクトルを計算するときに使用するクローラ間距離
  - アルミフレームでクローラ間距離を変えた場合この値を調整
- `l_wheel_radius (float, default: 0.03858)`
  - 左クローラの仮想タイヤ半径[m]
  - まっすぐ走らせてオドメトリがだんだん左に曲がっていく場合この値を少しだけ大きくするとよい（0.00002m刻み）
- `l_wheel_radius (float, default: 0.03858)`
  - 右クローラの仮想タイヤ半径[m]
  - まっすぐ走らせてオドメトリがだんだん右に曲がっていく場合この値を少しだけ大きくするとよい（0.00002m刻み）
- `reduction_ratio (float, default: 20.0)`
  - 減速比
  - ギヤボックスを変更した場合この値を調整
- `encoder_resolution (int, default: 30)`
  - モータのエンコーダ分解能
  - モータを交換した場合この値を調整

上記のパラメータはlaunchファイルで設定されています。

# Protocol
[cugo_ros_motorcontroller](https://github.com/CuboRex-Development/cugo_ros_motorcontroller/tree/pico-usb)と、ヘッダ8バイト・ボディ64バイトの合計72バイトから構成されるデータを通信しています。
ボディデータに格納されるデータの一覧は以下の通りになります。
ボディの残りの領域は今後拡張できるように確保されているだけで、現在は00を送受信しています。

### Arduinoドライバへの送信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|--------------------
TARGET_RPM_L   | float      | 4               | 0                           | RPM指令値(左モータ)
TARGET_RPM_R   | float      | 4               | 4                           | RPM指令値(右モータ)


### Arduinoドライバからの受信データ

Data Name      | Data Type  | Data Size(byte) | Start Address in PacketBody | Data Abstract
---------------|------------|-----------------|-----------------------------|-----------------
RECV_ENCODER_L | int32      | 4               | 0                           | 左エンコーダのカウント数
RECV_ENCODER_R | int32      | 4               | 4                           | 右エンコーダのカウント数


# Note

ご不明点がございましたら、[お問い合わせフォーム](https://cuborex.com/contact/)にてお問い合わせください。回答いたします。


# License
このプロジェクトはApache License 2.0のもと、公開されています。詳細はLICENSEをご覧ください。
