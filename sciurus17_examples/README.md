# sciurus17_examples

このパッケージはSciurus17 ROS 2パッケージのサンプルコード集です。

- [sciurus17\_examples](#sciurus17_examples)
  - [Setup](#setup)
    - [Using Sciurus17](#using-sciurus17)
    - [Using Gazebo](#using-gazebo)
    - [Using Mock Components](#using-mock-components)
  - [How to Run Examples](#how-to-run-examples)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [pose\_groupstate](#pose_groupstate)
    - [joint\_values](#joint_values)
    - [cartesian\_path](#cartesian_path)
    - [neck\_control](#neck_control)
    - [waist\_control](#waist_control)
    - [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
    - [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)
    - [head\_camera\_tracking](#head_camera_tracking)
    - [chest\_camera\_tracking](#chest_camera_tracking)
    - [color\_detection](#color_detection)
    - [point\_cloud\_detection](#point_cloud_detection)

## Setup

### Using Sciurus17

#### 1. Sciurus17本体をPCに接続する

Sciurus17本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

> [!NOTE]
> Sciurus17本体が接触しないように、十分なスペースを確保してください。

#### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`sciurus17_control`の[README](../sciurus17_control/README.md)を参照してください。

> [!NOTE]
> 正しく設定できていない場合、Sciurus17が動作しないので注意してください。

#### 3. move_groupとcontrollerを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)とcontroller (`sciurus17_control`)を起動します。

```sh
ros2 launch sciurus17_examples demo.launch.py
```

---

### Using Gazebo

#### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)とGazeboを起動します。

```sh
ros2 launch sciurus17_gazebo sciurus17_gazebo.launch.py
```

> [!NOTE]
> Sciurus17の手前にArUcoマーカ付きのBoxを置いたシミュレータ環境を使用する場合は次のコマンドを実行します。
> 
> ```sh
> ros2 launch sciurus17_gazebo sciurus17_with_aruco_cube.launch.py
> ```

> [!NOTE]
> Sciurus17の手前に青色のBoxを置いたシミュレータ環境を使用する場合は次のコマンドを実行します。
> 
> ```sh
> ros2 launch sciurus17_gazebo sciurus17_with_blue_cube.launch.py 
> ```

> [!NOTE]
> 頭部カメラや胸部カメラのシミュレーションを行わない場合は、`use_head_camera`、`use_chest_camera`オプションを`false`に設定します。
> 
> ```sh
> ros2 launch sciurus17_gazebo sciurus17_gazebo.launch.py use_head_camera:=false use_chest_camera:=false
> ```

---

### Using Mock Components

#### 1. move_groupとcontrollerを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)とcontroller (`sciurus17_control`)を起動します。

```sh
ros2 launch sciurus17_examples demo.launch.py use_mock_components:=true
```

Mock Componentsではカメラを使ったサンプルを実行することはできません。

## How to Run Examples

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch sciurus17_examples example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。


> [!NOTE]
> Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。
> 
> ```sh
> ros2 launch sciurus17_examples example.launch.py example:='gripper_control' use_sim_time:='true'
> ```

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper\_control](#gripper_control)
- [pose\_groupstate](#pose_groupstate)
- [joint\_values](#joint_values)
- [cartesian\_path](#cartesian_path)
- [neck\_control](#neck_control)
- [waist\_control](#waist_control)
- [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
- [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)
- [head\_camera\_tracking](#head_camera_tracking)
- [chest\_camera\_tracking](#chest_camera_tracking)
- [point\_cloud\_detection](#point_cloud_detection)

実行できるサンプルの一覧は、`example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
ros2 launch sciurus17_examples example.launch.py -s
```

### gripper_control

ハンドを開閉させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='gripper_control'
```

[Back to example list](#examples)

---

### pose_groupstate

group_stateを使うコード例です。

SRDFファイル[sciurus17_moveit_config/config/sciurus17.srdf](../sciurus17_moveit_config/config/sciurus17.srdf)に記載されている`two_arm_init_pose`と`two_arm_push_forward_pose`の姿勢に移行します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='pose_groupstate'
```

[Back to example list](#examples)

---

### joint_values

左側のアームのジョイント角度を１つずつ変化させるコード例です。

SRDFファイル[sciurus17_moveit_config/config/sciurus17.srdf](../sciurus17_moveit_config/config/sciurus17.srdf)に記載されている`l_arm_init_pose`から、各関節角度を15度ずつ順番に変化させます。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='joint_values'
```

[Back to example list](#examples)

---

### cartesian_path

[Cartesian Path](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html#cartesian-paths)
を生成し、手先で円を描くコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='cartesian_path'
```

[Back to example list](#examples)

---

### neck_control

首を上下左右へ動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='neck_control'
```

[Back to example list](#examples)

---

### waist_control

腰を左右へひねる動作をするコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='waist_control'
```

[Back to example list](#examples)

---

### pick_and_place_right_arm_waist

右手でターゲットを掴んで動かすコード例です。腰の回転も使用します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='pick_and_place_right_arm_waist'
```

[Back to example list](#examples)

---

### pick_and_place_left_arm

左手でターゲットを掴んで動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='pick_and_place_left_arm'
```

[Back to example list](#examples)

---

### head_camera_tracking

頭部カメラ映像を用いてオレンジ色の物体を追従するコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples head_camera_tracking.launch.py
```

> [!NOTE]
> Gazeboで実行する場合は動作環境によってうまく追従しない場合があります。
> カメラ解像度やサンプルコード内の追従速度ゲインを調整してください。

[Back to example list](#examples)

---

### chest_camera_tracking

胸部カメラ映像を用いてオレンジ色の物体を追従するコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples chest_camera_tracking.launch.py
```

> [!NOTE]
> Gazeboで実行する場合は動作環境によってうまく追従しない場合があります。
> カメラ解像度やサンプルコード内の追従速度ゲインを調整してください。

[Back to example list](#examples)

---

### color_detection

特定の色の物体を検出して掴むコード例です。

- デフォルトでは青い物体の位置をtfのフレームとして配信されます。
- tfの`frame_id`は`target_0`です。
- 色の検出にはOpenCVを使用しています。
- 検出した物体の距離は深度画像から取得します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples camera_example.launch.py example:='color_detection'
```

[Back to example list](#examples)

---

### point_cloud_detection

点群から物体を検出して掴むコード例です。

- 検出された物体位置はtfのフレームとして配信されます。
- tfの`frame_id`は検出された順に`target_0`、`target_1`、`target_2`…に設定されます。
- 掴む対象はSciurus17前方の0.3 mの範囲にある`target_0`に設定されています。
- 物体検出には[Point Cloud Library](https://pointclouds.org/)を使用しています。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples camera_example.launch.py example:='point_cloud_detection'
```

[Back to example list](#examples)

---
