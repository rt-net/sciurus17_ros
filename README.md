[English](README.md) | [日本語](README.ja.md)

sciurus17_ros
====

[![Build Status](https://travis-ci.com/rt-net/sciurus17_ros.svg?branch=master)](https://travis-ci.com/rt-net/sciurus17_ros)

![sciurus17_gazebo](https://github.com/rt-net/sciurus17_ros/blob/images/images/sciurus17_gazebo.png "sciurus17_gazebo")

ROS Packages for Sciurus17.

Product page:  
[https://www.rt-net.jp/products/sciurus17](https://www.rt-net.jp/products/sciurus17?lang=en)

ROS Wiki:  
[https://wiki.ros.org/sciurus17](https://wiki.ros.org/sciurus17)

Examples:  
[sciurus17_examples](https://github.com/rt-net/sciurus17_ros/tree/master/sciurus17_examples)

## System Requirements

These packages have been developed and tested on ROS Kinetic & Melodic.
Please see below for details.

- ROS Kinetic
  - OS: Ubuntu 16.04.5 LTS
  - ROS Distribution: Kinetic Kame 1.12.14
  - Rviz 1.12.17
  - MoveIt! 0.9.17
  - Gazebo 7.0.0
- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.3
  - Rviz 1.12.16
  - MoveIt! 1.13.3
  - Gazebo 9.0.0
  
## Installation

### Build from source

- Install ROS environments. Please see [ROS Wiki](http://wiki.ros.org/melodic/Installation/Ubuntu).

- Install [Intel RealSense SDK 2.0](https://github.com/IntelRealSense/librealsense).
  - Please refer to [installation documents](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md) 
  to install SDK(librealsense2-\*).

- Download and build a ROS package [realsense2_camera](http://wiki.ros.org/realsense2_camera).
  - `sciurus17_ros` supports the package version 2.2.0. Please refer to [`Releases`](https://github.com/IntelRealSense/realsense-ros/releases) for the package version.
  - Please select correct versions of *librealsense* and *realsense2_camera*.

- Download the packages for Sciurus17 using `git`.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_ros.git
  ```

- Install package dependencies.

  ```bash
  cd ~/catkin_ws/src

  # package for sciurus17_gazebo
  git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git

  rosdep install -r -y --from-paths . --ignore-src
  ```

- Build packages using `catkin_make`.

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

**NOTE:** Please refer to 
[sciurus17_examples/README.md](./sciurus17_examples/README.md)
to install OpenCV for image processing examples execution.

## Device Setup

Apply udev rules with following commands to enable communication between `sciurus17_control` and Sciurus17.

```bash
roscd sciurus17_tools/scripts/
./create_udev_rules
```
Then reboot a PC and new device name `/dev/sciurus17spine` will be created.

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
The MoveIt! packages and Gazebo require this package.

### sciurus17_moveit_config

This package includes configuration files for MoveIt!.

To launch the MoveIt! demonstration with Rviz:

```bash
roslaunch sciurus17_moveit_config demo.launch
```

### sciurus17_msgs

This package defines ros message types for Sciurus17.

### sciurus17_vision

This package includes launch files for camera nodes.

### sciurus17_bringup

This package includes launch files for startup of Sciurus17.

### sciurus17_tools

This package includes option tools for Sciurus17.

If head camera (RealSense) is unstable, please run following command before startup.

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

