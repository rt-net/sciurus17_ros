[English](README.en.md) | [日本語](README.md)

# sciurus17_ros

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml)

ROS 2 package suite of Sciurus17.

![sciurus17_gazebo](https://rt-net.github.io/images/sciurus17/sciurus17_gazebo2.png "sciurus17_gazebo")

## Table of Contents

- [sciurus17\_ros](#sciurus17_ros)
  - [Table of Contents](#table-of-contents)
  - [Supported ROS 2 distributions](#supported-ros-2-distributions)
    - [ROS 1](#ros-1)
  - [Requirements](#requirements)
  - [Installation](#installation)
    - [Build from source](#build-from-source)
    - [Device setup](#device-setup)
  - [Quick Start](#quick-start)
  - [Packages](#packages)
  - [License](#license)
  - [開発について](#開発について)

## Supported ROS 2 distributions

- Humble

### ROS 1

- [Melodic](https://github.com/rt-net/sciurus17_ros/tree/master)
- [Noetic](https://github.com/rt-net/sciurus17_ros/tree/master)

## Requirements

- Sciurus17
  - [製品ページ](https://www.rt-net.jp/products/sciurus17)
  - [ウェブショップ](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3895)
- Linux OS
  - Ubuntu 22.04
- ROS
  - [Humble Hawksbill](https://docs.ros.org/en/humble/Installation.html)

## Installation

### Build from source

```sh
# Setup ROS environment
source /opt/ros/humble/setup.bash

# Download packages
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/rt-net/sciurus17_ros.git
git clone -b ros2 https://github.com/rt-net/sciurus17_description.git

# Install dependencies
rosdep install -r -y -i --from-paths .

# Build & Install
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```

### Device setup

次の方法で`sciurus17_control`が実機と通信するために用いるUSBシリアル変換デバイス名を固定します。

```sh
ros2 run sciurus17_tools create_udev_rules
```

実行後に再起動しSciurus17を接続すると`/dev/sciurus17spine`が作成されるようになります。

## Quick Start

```sh
# Connect Sciurus17 to PC, then
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples demo.launch.py
```

```sh
# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
```

詳細は[sciurus17_examples](./sciurus17_examples/README.md)を参照してください。

## Packages

- sciurus17_control
  - [README](./sciurus17_control/README.md)
  - Sciurus17の制御を行うパッケージです
- sciurus17_examples
  - [README](./sciurus17_examples/README.md)
  - Sciurus17のサンプルコード集です  
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

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[sciurus17_description](https://github.com/rt-net/sciurus17_description/tree/ros2)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[sciurus17_description/LICENSE](https://github.com/rt-net/sciurus17_description/blob/ros2/LICENSE)を参照してください。

## 開発について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。
