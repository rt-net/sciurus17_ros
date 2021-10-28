[English](README.en.md) | [日本語](README.md)

sciurus17_ros
====

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/sciurus17_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

![sciurus17_gazebo](https://rt-net.github.io/images/sciurus17/sciurus17_gazebo.png "sciurus17_gazebo")

ROS Packages for Sciurus17.

Product page:  
[https://www.rt-net.jp/products/sciurus17](https://www.rt-net.jp/products/sciurus17?lang=en)

ROS Wiki:  
[https://wiki.ros.org/sciurus17](https://wiki.ros.org/sciurus17)

Examples:  
[sciurus17_examples](https://github.com/rt-net/sciurus17_ros/tree/master/sciurus17_examples)

## System Requirements

These packages have been developed and tested on ROS Melodic & Noetic.
Please see below for details.

- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.9
  - Rviz 1.13.19
  - MoveIt 1.0.8
  - Gazebo 9.0.0
- ROS Noetic
  - OS: Ubuntu 20.04.3 LTS
  - ROS Distribution: Noetic Ninjemys 1.15.8
  - Rviz 1.14.10
  - MoveIt 1.1.5
  - Gazebo 11.5.1

## Installation

### Build from source

- Install ROS environments. Please see [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

- Download the packages for Sciurus17 using `git`.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_ros.git
  ```

- Install package dependencies.

  ```bash
  cd ~/catkin_ws/src

  rosdep install -r -y --from-paths . --ignore-src
  ```

- Build packages using `catkin_make`.

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### Device Setup

Apply udev rules with the following commands to enable communication between `sciurus17_control` and Sciurus17.

```bash
roscd sciurus17_tools/scripts/
./create_udev_rules
```
Reboot the PC after running the script to update the udev rules.
After rebooting, the new device `/dev/sciurus17spine` will be created.

## About Sciurus17 packages

### sciurus17_control

This package controls Sciurus17 using `Dynamixel SDK C++ Library`
which can install by `rosdep install` command.
Read and write permissions on `/dev/sciurus17spine` 
are required for communication between the package and Sciurus17.

The device name of serial port and parameters of Dynamixel servo motors are listed in:

- `config/sciurus17_control1.yaml`
- `config/sciurus17_control2.yaml`
- `config/sciurus17_control3.yaml`

If this package did not find the serial port, the package switches its control mode to Dummy Joint Mode from Normal Mode
and republishes target joint values as servo angle values.
This is useful for debugging of motion control without Sciurus17 hardware.

At startup, this package moves the Sciurus17 to Home Position in 5 seconds.
At shutdown, this package decreases PID gains of the servo motors to stop motion safely.

### sciurus17_description

This package defines Sciurus17 model data including links and joints.
The MoveIt packages and Gazebo require this package.

### sciurus17_moveit_config

This package includes configuration files for MoveIt.

To launch the MoveIt demonstration with Rviz:

```bash
roslaunch sciurus17_moveit_config demo.launch
```

### sciurus17_msgs

This package defines ROS message types for Sciurus17.

### sciurus17_vision

This package includes launch files for camera nodes.

### sciurus17_bringup

This package includes launch files for startup of Sciurus17.

### sciurus17_tools

This package includes option tools for Sciurus17.

If the head camera (RealSense) is unstable, please run the following command before startup.

```bash
rosrun sciurus17_tools realsense_hwreset
```

### sciurus17_gazebo

This package includes Gazebo simulation environments for Sciurus17.

To simulate CRANE-X7 on the table:

```bash
roslaunch sciurus17_gazebo sciurus17_with_table.launch
```

**NOTE:** First launch will take long time to download gazebo models. Please wait until Gazebo screen is displayed.

### sciurus17_examples

This package includes example codes for Sciurus17.
Please refer to [./sciurus17_examples/README.md](./sciurus17_examples/README.md).

### Proprietary Rights

Sciurus17 is an upper body robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use Sciurus17 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us. 

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS. 

