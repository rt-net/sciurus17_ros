[English](README.en.md) | [日本語](README.md)

# sciurus17_ros

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/workflows/industrial_ci/badge.svg?branch=ros2)](https://github.com/rt-net/sciurus17_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS 2 package suite of Sciurus17.

![sciurus17_gazebo](https://rt-net.github.io/images/sciurus17/sciurus17_gazebo.png "sciurus17_gazebo")

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

## Supported ROS 2 distributions

- Humble

### ROS 1

- [Melodic](https://github.com/rt-net/sciurus17_ros/tree/master)
- [Noetic](https://github.com/rt-net/sciurus17_ros/tree/master)

## Requirements

- Sciurus17
  - [Product page](https://www.rt-net.jp/products/sciurus17)
  - [](https://www.rt-shop.jp/index.php?main_page=product_info&products_id=3895)
  - [ROS Wiki](https://wiki.ros.org/sciurus17)
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

Apply udev rules with the following commands to enable communication between `sciurus17_control` and Sciurus17.

```sh
ros2 run sciurus17_tools create_udev_rules
```

Reboot the PC after running the script to update the udev rules.
After rebooting, the new device `/dev/sciurus17spine` will be created.

## Quick Start

```sh
# Connect Sciurus17 to PC, then
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples demo.launch.py

# Terminal 2
source ~/ros2_ws/install/setup.bash
ros2 launch sciurus17_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
```

詳細は[sciurus17_examples](./sciurus17_examples/README.md)を参照してください。
Please refer to [./sciurus17_examples/README.en.md](./sciurus17_examples/README.en.md).

## Packages

- sciurus17_control
  - [README](./sciurus17_control/README.md)
  - This package includes a hardware driver for Sciurus17.
- sciurus17_examples
  - [README](./sciurus17_examples/README.en.md)
  - This package includes example codes for Sciurus17.
- sciurus17_gazebo
  - This package includes Gazebo simulation environments for Sciurus17.
- sciurus17_moveit_config
  - This package includes configuration files for `moveit2`.
- sciurus17_tools
  - This package includes option tools for Sciurus17.
- sciurus17_vision
  - This package includes launch files for camera nodes.
  - This package does not support ROS 2 yet.
- sciurus17_description (external package)
  - [README](https://github.com/rt-net/sciurus17_description/blob/ros2/README.en.md)
  - This package includes a model data (xacro) of Sciurus17.

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

The sciurus17_ros depends on [sciurus17_description](https://github.com/rt-net/sciurus17_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE) applies to the package.
