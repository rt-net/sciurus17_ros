# sciurus17_examples

このパッケージはSciurus17 ROS 2パッケージのサンプルコード集です。

- [sciurus17\_examples](#sciurus17_examples)
  - [Setup](#setup)
    - [Using Sciurus17](#using-sciurus17)
    - [Using Gazebo](#using-gazebo)
    - [Using Mock Components](#using-mock-components)
  - [How to Run](#how-to-run)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [neck\_control](#neck_control)
    - [waist\_control](#waist_control)
    - [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
    - [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)
    - [head\_camera\_tracking](#head_camera_tracking)
    - [chest\_camera\_tracking](#chest_camera_tracking)
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

### Using Gazebo

#### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)とGazeboを起動します。

```sh
ros2 launch sciurus17_gazebo sciurus17_with_table.launch.py
```

頭部カメラや胸部カメラのシミュレーションを行わない場合は、`use_head_camera`、`use_chest_camera`オプションを`false`に設定します。

```sh
ros2 launch sciurus17_gazebo sciurus17_with_table.launch.py use_head_camera:=false use_chest_camera:=false
```

### Using Mock Components

#### 1. move_groupとcontrollerを起動する

次のコマンドでmove_group (`sciurus17_moveit_config`)とcontroller (`sciurus17_control`)を起動します。

```sh
ros2 launch sciurus17_examples demo.launch.py use_mock_components:=true
```

Mock Componentsではカメラを使ったサンプルを実行することはできません。

## How to Run

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

[back to example list](#examples)

---

### neck_control

首を上下左右へ動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='neck_control'
```

[back to example list](#examples)

---

### waist_control

腰を左右へひねる動作をするコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='waist_control'
```

[back to example list](#examples)

---

### pick_and_place_right_arm_waist

右手でターゲットを掴んで動かすコード例です。腰の回転も使用します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='pick_and_place_right_arm_waist'
```

[back to example list](#examples)

---

### pick_and_place_left_arm

左手でターゲットを掴んで動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples example.launch.py example:='pick_and_place_left_arm'
```

[back to example list](#examples)

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

[back to example list](#examples)

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

[back to example list](#examples)

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

[back to example list](#examples)

---
