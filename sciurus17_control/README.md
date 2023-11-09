# sciurus17_control

このパッケージは[ros2_control](https://github.com/ros-controls/ros2_control)
をベースにした、Sciurus17 のコントローラパッケージです。

## ros2_control関連ファイル

- `sciurus17_control::Sciurus17Hardware (sciurus17_hardware)`
  - 本パッケージがエクスポートする[Hardware Components](https://control.ros.org/master/doc/getting_started/getting_started.html#hardware-components)です
  - Sciurus17実機と通信します
  - sciurus17_descriptionから読み込まれます
- [launch/sciurus17_control.launch.py](./launch/sciurus17_control.launch.py)
  - [Controller Manager](https://control.ros.org/master/doc/getting_started/getting_started.html#controller-manager)とコントローラを起動するlaunchファイルです
- [config/sciurus17_controllers.yaml](./config/sciurus17_controllers.yaml)
  - Controller Managerのパラメータファイルです

## 実機のセットアップ

`sciurus17_hardware`がSciurus17実機と通信するために、
PCとSciurus17の設定が必要です。

### USB通信ポートの設定

次の方法で`sciurus17_control`が実機と通信するために用いるUSBシリアル変換デバイス名を固定します。

```sh
ros2 run sciurus17_tools create_udev_rules
```

実行後に再起動しSciurus17を接続すると`/dev/sciurus17spine`が作成されるようになります。

## ノードの起動

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

## Controller Managerのパラメータ

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

### 制御周期

`update_rate`は制御周期を設定します。

### コントローラ

Sciurus17の腕の制御用に`right_arm_controller`と`left_arm_controller`を、
グリッパの制御用に`right_gripper_controller`と`left_gripper_controller`を設定しています。

## sciurus17_hardwareのパラメータ

`sciurus17_hardware`のパラメータは
`sciurus17_description/urdf/sciurus17.urdf.xacro`
で設定しています。

```xml
  <xacro:arg name="port_name" default="/dev/sciurus17spine" />
  <xacro:arg name="baudrate" default="3000000" />
  <xacro:arg name="timeout_seconds" default="1.0" />
  <xacro:arg name="manipulator_config_file_path" default="" />
```

### USB通信ポート

`port_name`はSciurus17との通信に使用するUSB通信ポートを設定します。

### ボーレート

`baudrate`はSciurus17に搭載したDynamixelとの通信ボーレートを設定します。

デフォルト値には`3000000` (3 Mbps)を設定しています。

### 通信タイムアウト

`timeout_seconds`は通信タイムアウト時間（秒）を設定します。

`sciurus17_hardware`は、一定時間（デフォルト1秒間）通信に失敗し続けると、
read/write動作を停止します。
USBケーブルや電源ケーブルが抜けた場合等に有効です。

### RTマニピュレータC++ライブラリ用の設定ファイルパス

`sciurus17_hardware`は、Sciurus17と通信するために
[RTマニピュレータC++ライブラリ](https://github.com/rt-net/rt_manipulators_cpp)
を使用しています。


`manipulatcor_config_file_path`は
ライブラリが読み込むサーボ設定ファイルへのパスを設定します。

---

[back to top](#sciurus17_control)
