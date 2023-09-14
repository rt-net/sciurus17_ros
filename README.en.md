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
[sciurus17_examples](https://github.com/rt-net/sciurus17_ros/tree/master/sciurus17_examples/README.en.md)

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

- Download [sciurus17_description](https://github.com/rt-net/sciurus17_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE) applies to the package.

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_description.git
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

### Upgrading to v2.x.x from v1.0.0 or earlier

Please see 
https://github.com/rt-net/sciurus17_ros/issues/134 
for details of differences in the versions.

Update the package with the following commands:

```bash
# Update sciurus17_ros
cd ~/catkin_ws/src/sciurus17_ros
git pull origin master

# Download sciurus17_description package
cd ~/catkin_ws/src
git clone https://github.com/rt-net/sciurus17_description.git
rosdep install -r -y --from-paths . --ignore-src

# Clean up the workspace and rebuild packages
# Note that other ROS packages in the workspace will also be rebuilt.
cd ~/catkin_ws
rm -r build devel
catkin_make
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

```bash
roslaunch sciurus17_gazebo sciurus17_with_table.launch
```

**NOTE:** First launch will take long time to download gazebo models. Please wait until Gazebo screen is displayed.

### sciurus17_examples

This package includes example codes for Sciurus17.
Please refer to [./sciurus17_examples/README.en.md](./sciurus17_examples/README.en.md).

## License

(C) 2018 RT Corporation \<support@rt-net.jp\>

This repository is licensed under the Apache License, Version 2.0, see [LICENSE](./LICENSE).  
Unless attributed otherwise, everything in this repository is under the Apache License, Version 2.0.

The sciurus17_ros depends on [sciurus17_description](https://github.com/rt-net/sciurus17_description) package.
The RT Corporation's [NON-COMMERCIAL LICENSE](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE) applies to the package.
