[English](README.en.md) | [日本語](README.md)

# sciurus17_ros

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml)

ROS 2でSciurusS17を動作させるパッケージです。

![sciurus17\_gazebo](https://rt-net.github.io/images/sciurus17/sciurus17_gazebo2.png "sciurus17_gazebo")

## Table of Contents

- [sciurus17\_ros](#sciurus17_ros)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS 2 distributions](#supported-ros-2-distributions)
  - [Requirements](#requirements)
  - [Installation](#installation)
  - [Quick Start](#quick-start)
  - [Packages](#packages)
  - [How ot Use Examples](#how-to-use-examples)
  - [License](#license)
  - [Contributing](#contributing)

## Supported ROS 2 distributions

- [Humble Hawksbill](https://github.com/rt-net/sciurus17_ros/tree/humble)
- [Jazzy Jalisco](https://github.com/rt-net/sciurus17_ros/tree/jazzy)

## Requirements

- Sciurus17
  - [製品ページ](https://www.rt-net.jp/products/sciurus17)
  - [ウェブショップ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3895)
- Linux OS
  - Ubuntu 24.04
- ROS 2
  - [Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation.html)

## Installation

### Source Build

```sh
# Download packages
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b $ROS_DISTRO https://github.com/rt-net/sciurus17_ros.git
git clone -b $ROS_DISTRO https://github.com/rt-net/sciurus17_description.git

# Install dependencies
rosdep install -r -y -i --from-paths .
sudo apt install libpcl-dev

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

## Quick Start

### Device Setup

以下のコマンドで`sciurus17_control`が実機と通信するために用いるUSBシリアル変換デバイス名を固定します。
実行後に再起動しSciurus17を接続すると`/dev/sciurus17spine`が作成されるようになります。

```sh
ros2 run sciurus17_tools create_udev_rules
```

### Run

次に、以下のコマンドを実行すると、Sciurus17がグリッパ開閉動作をします。

```sh
# Connect Sciurus17 to PC, then
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples demo.launch.py

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
```

## Packages

- sciurus17_control
  - [README](./sciurus17_control/README.md)
  - Sciurus17の制御を行うパッケージです
- sciurus17_examples
  - [README](./sciurus17_examples/README.md)
  - Sciurus17のサンプルコード集です  
- sciurus17_examples_py
  - [README](./sciurus17_examples_py/README.md)
  - Sciurus17のPythonサンプルコード集です  
- sciurus17_gazebo
  - Sciurus17のGazeboシミュレーションパッケージです
- sciurus17_moveit_config
  - Sciurus17の`moveit2`設定ファイルです
- sciurus17_tools
  - Sciurus17を活用するためのオプションツールをまとめたパッケージです
- sciurus17_vision
  - カメラのlaunchファイルや画像認識を行うノードを定義するパッケージです
  - 胸部カメラのキャリブレーションパラメータファイルは[chest_camera_info.yaml](./sciurus17_vision/config/chest_camera_info.yaml)です
- sciurus17_description (外部パッケージ)
  - [README](https://github.com/rt-net/sciurus17_description/blob/ros2/README.md)
  - Sciurus17のモデルデータ（xacro）を定義するパッケージです

## How to Use Examples

サンプルプログラムは、C++とPythonの両方を用意しています。詳しくは、以下のリンクをご覧ください。

### C++

- [sciurus17_examples](./sciurus17_examples/README.md)
  - Examples
    - gripper_control
    - neck_control
    - waist_control
    - pick_and_place_right_arm_waist
    - pick_and_place_left_arm
    - head_camera_tracking
    - chest_camera_tracking
    - point_cloud_detection

### Python

- [crane_x7_examples_py](./crane_x7_examples_py/README.md)
  - Examples
    - gripper_control
    - neck_control
    - waist_control
    - pick_and_place_right_arm_waist
    - pick_and_place_left_arm

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルにライセンスが明記されている場合、そのライセンスに従います。
特に明記がない場合は、Apache License, Version 2.0に基づいて公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[sciurus17_description](https://github.com/rt-net/sciurus17_description/tree/ros2)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[sciurus17_description/LICENSE](https://github.com/rt-net/sciurus17_description/blob/ros2/LICENSE)を参照してください。

## Contributing

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。
