# sciurus17_examples_py

このパッケージはSciurus17 ROS 2パッケージのPythonサンプルコード集です。

- [sciurus17\_examples](#sciurus17_examples)
  - [起動方法](#起動方法)
  - [サンプルプログラムを実行する](#サンプルプログラムを実行する)
    - [Gazeboでサンプルプログラムを実行する場合](#gazeboでサンプルプログラムを実行する場合)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [neck\_control](#neck_control)
    - [waist\_control](#waist_control)
    - [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
    - [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)

## 起動方法
Sciurus17の起動方法は[sciurus17_examplesのREADME](../sciurus17_examples/README.md)を参照してください。

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

### Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control' use_sim_time:='true'
```

## Examples

`demo.launch`を実行している状態で各サンプルを実行できます。

- [gripper\_control](#gripper_control)
- [pose\_groustate](#pose_groupstate)
- [neck\_control](#neck_control)
- [waist\_control](#waist_control)
- [pick\_and\_place\_right\_arm\_waist](#pick_and_place_right_arm_waist)
- [pick\_and\_place\_left\_arm](#pick_and_place_left_arm)

実行できるサンプルの一覧は、`example.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
ros2 launch sciurus17_examples_py example.launch.py -s
```

### gripper_control

ハンドを開閉させるコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='gripper_control'
```

[back to example list](#examples)

---

### pose_groupstate

group_stateを使うコード例です。

SRDFファイル[sciurus17_moveit_config/config/sciurus17.srdf](../sciurus17_moveit_config/config/sciurus17.srdf)に記載されている`two_arm_init_pose`と`two_arm_push_forward_pose`の姿勢に移行します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='pose_groupstate'
```

### neck_control

首を上下左右へ動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='neck_control'
```

[back to example list](#examples)

---

### waist_control

腰を左右へひねる動作をするコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='waist_control'
```

[back to example list](#examples)

---

### pick_and_place_right_arm_waist

右手でターゲットを掴んで動かすコード例です。腰の回転も使用します。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='pick_and_place_right_arm_waist'
```

[back to example list](#examples)

---

### pick_and_place_left_arm

左手でターゲットを掴んで動かすコード例です。

次のコマンドを実行します。

```sh
ros2 launch sciurus17_examples_py example.launch.py example:='pick_and_place_left_arm'
```

[back to example list](#examples)

---
