# sciurus17_control

このパッケージは[ros2_control](https://github.com/ros-controls/ros2_control)
をベースにした、Sciurus17 のコントローラパッケージです。

## Table of Contents

- [sciurus17\_control](#sciurus17_control)
  - [ros2\_control Files](#ros2_control-files)
- [Setup](#setup)
  - [USB Port Configuration](#usb-port-configuration)
- [How to Launch Nodes](#how-to-launch-nodes)
- [Controller Manager Parameters](#controller-manager-parameters)
  - [Control Cycle](#control-cycle)
  - [Controllers](#controllers)
- [sciurus17\_hardware Parameters](#sciurus17_hardware-parameters)
  - [USB Port](#usb-port)
  - [Baudrate](#baudrate)
  - [Communication Timeout](#communication-timeout)
  - [Configuration File Paths for RT Manipulator C++ Library](#configuration-file-paths-for-rt-manipulator-c-library)

## ros2_control Files

- `sciurus17_control::Sciurus17Hardware (sciurus17_hardware)`
  - 本パッケージがエクスポートする[Hardware Components](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components)です
  - Sciurus17実機と通信します
  - sciurus17_descriptionから読み込まれます
- [launch/sciurus17_control.launch.py](./launch/sciurus17_control.launch.py)
  - [Controller Manager](https://control.ros.org/master/doc/getting_started/getting_started.html#controller-manager)とコントローラを起動するlaunchファイルです
- [config/sciurus17_controllers.yaml](./config/sciurus17_controllers.yaml)
  - Controller Managerのパラメータファイルです

## Setup

`sciurus17_hardware`がSciurus17実機と通信するために、
PCとSciurus17の設定が必要です。

### USB Port Configuration

次の方法で`sciurus17_control`が実機と通信するために用いるUSBシリアル変換デバイス名を固定します。

```sh
ros2 run sciurus17_tools create_udev_rules
```

実行後に再起動しSciurus17を接続すると`/dev/sciurus17spine`が作成されるようになります。

## How to Launch Nodes

`sciurus17_control.launch.py`を実行すると、`Controller Manager`ノードが起動し、
以下のコントローラが読み込まれます。

- right_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- right_gripper_controller (`position_controllers/GripperActionController`)
- left_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- left_gripper_controller (`position_controllers/GripperActionController`)
- neck_controller (`joint_trajectory_controller/JointTrajectoryController`)
- waist_yaw_controller (`joint_trajectory_controller/JointTrajectoryController`)
- joint_state_broadcaster (`joint_state_broadcaster/JointStateBroadcaster`)

ノードが起動した後、
次のコマンドでジョイント角度情報（`joint_states`）を表示できます

```sh
ros2 topic echo /joint_states
```

## Controller Manager Parameters

`Controller Manager`のパラメータは
[config/sciurus17_controllers.yaml](./config/sciurus17_controllers.yaml)
で設定しています。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    right_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    right_gripper_controller:
      type: position_controllers/GripperActionController
```

### Control Cycle

`update_rate`は制御周期を設定します。

### Controllers

Sciurus17の各関節を制御するコントローラの設定ができます。コントローラの名称と役割の対応は以下のとおりです。

- right_arm_controller
  - 右腕制御用コントローラ
- right_gripper_controller
  - 右グリッパ制御用コントローラ
- left_arm_controller
  - 左腕制御用コントローラ
- left_gripper_controller
  - 左グリッパ制御用コントローラ
- neck_controller
  - 首制御用コントローラ
- waist_yaw_controller
  - 腰制御用コントローラ

## sciurus17_hardware Parameters

`sciurus17_hardware`のパラメータは
`sciurus17_description/urdf/sciurus17.urdf.xacro`
で設定しています。

```xml
  <xacro:arg name="port_name" default="/dev/sciurus17spine" />
  <xacro:arg name="baudrate" default="3000000" />
  <xacro:arg name="timeout_seconds" default="1.0" />
  <xacro:arg name="manipulator_config_file_path" default="" />
```

### USB Port

`port_name`はSciurus17との通信に使用するUSB通信ポートを設定します。

### Baudrate

`baudrate`はSciurus17に搭載したDynamixelとの通信ボーレートを設定します。

デフォルト値には`3000000` (3 Mbps)を設定しています。

### Communication Timeout

`timeout_seconds`は通信タイムアウト時間（秒）を設定します。

`sciurus17_hardware`は、一定時間（デフォルト1秒間）通信に失敗し続けると、
read/write動作を停止します。
USBケーブルや電源ケーブルが抜けた場合等に有効です。

### Configuration File Paths for RT Manipulator C++ Library

`sciurus17_hardware`は、Sciurus17と通信するために
[RTマニピュレータC++ライブラリ](https://github.com/rt-net/rt_manipulators_cpp)
を使用しています。


`manipulatcor_config_file_path`は
ライブラリが読み込むサーボ設定ファイルへのパスを設定します。

---

[Back to top](#sciurus17_control)
