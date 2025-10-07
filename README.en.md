[English](README.en.md) | [日本語](README.md)

# sciurus17_ros

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_ros/actions/workflows/industrial_ci.yml)

This is a ROS 2 package suite for the Sciurus17.

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
  - [Product page](https://www.rt-net.jp/products/sciurus17)
  - [Web shop](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3895)
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

Sample programs are available in both C++ and Python. See the links below for details.

```sh
ros2 run sciurus17_tools create_udev_rules
```

Reboot the PC after running the script to update the udev rules.
After rebooting, the new device `/dev/sciurus17spine` will be created.

### Run

The following command makes Sciurus17 open and close its gripper.

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

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

The sciurus17_ros depends on [sciurus17_description](https://github.com/rt-net/sciurus17_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE) applies to the package.

## Contributing

- This software is open source, but its development is not open.
- This software is essentially provided as open source software on an “AS IS” (in its current state) basis.
- No free support is available for this software.
- Requests for bug fixes and corrections of typographical errors are always accepted; however, requests for additional features will be subject to our internal guidelines. For further details, please refer to the [Contribution Guidelines](https://github.com/rt-net/.github/blob/master/CONTRIBUTING.md).
