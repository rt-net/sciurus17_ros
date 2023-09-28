[English](README.en.md) | [日本語](README.md)

# sciurus17_examples

Sciurus17のためのパッケージ、 `sciurus17` で用いるサンプルをまとめたパッケージです。

## システムの起動方法

Sciurus17の頭部カメラ、胸部カメラ、制御信号の各ケーブルを制御用パソコンへ接続します。  
本体の電源をONにしカメラが接続されていることを確認します。  
Terminalを開き、 `sciurus17_bringup` の `sciurus17_bringup.launch` を起動します。このlaunchファイルには次のオプションが用意されています。

- use_rviz (default: true)  
Rvizを使用する/使用しない
- use_head_camera (default: true)  
頭部カメラを使用する/使用しない
- use_chest_camera (default: true)  
胸部カメラを使用する/使用しない

### 簡易シミュレータを使う場合

実機無しで動作を確認する場合、制御信号のケーブルを接続しない状態で次のコマンドを実行します。  

```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

物理演算やセンサを含めたシミュレーションは、
後述の[「Gazeboを使う場合」](#gazeboを使う場合)の手順に従ってください。

### 実機を使う場合

実機で動作を確認する場合、制御信号のケーブルを接続し取り扱い説明書に従いモータパワーをONにした状態で次のコマンドを実行します。  

```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

### カメラを使用しない場合

次のようにオプションを指定するとカメラを使用しない状態で起動します。

```
roslaunch sciurus17_bringup sciurus17_bringup.launch use_head_camera:=false use_chest_camera:=false
```

### rvizを使用しない場合

次のようにオプションを指定するとrvizによる画面表示を使用しない状態で起動します。画面表示を省略することで制御用パソコンの負荷を下げることができます。  

```
roslaunch sciurus17_bringup sciurus17_bringup.launch use_rviz:=false
```

### Gazeboを使う場合

次のコマンドで起動します。実機との接続やsciurus17_bringupの実行は必要ありません。

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch

# rvizを使用しない場合
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_rviz:=false
```

## Run Examples

`sciurus17_bringup.launch`を実行している状態で各サンプルを実行できます。  

- [gripper_action_example](#gripper_action_example)
- [neck_joint_trajectory_example](#neck_joint_trajectory_example)
- [waist_joint_trajectory_example](#waist_joint_trajectory_example)
- [pick_and_place_demo](#pick_and_place_demo)
- [hand_position_publisher](#hand_position_publisher)
- [head_camera_tracking](#head_camera_tracking)
- [chest_camera_tracking](#chest_camera_tracking)
- [depth_camera_tracking](#depth_camera_tracking)
- [preset_pid_gain_example](#preset_pid_gain_example)
- [box_stacking_example](#box_stacking_example)
- [current_control_right_arm](#current_control_right_arm)
- [current_control_left_wrist](#current_control_left_wrist)

### gripper_action_example

両腕のハンドを開閉させるコード例です。   
次のコマンドで26度まで開いて閉じる動作を実行します。

```
rosrun sciurus17_examples gripper_action_example.py
```

<img src= https://rt-net.github.io/images/sciurus17/gazebo_gripper_example.gif width=500px />

#### Videos

[![](http://img.youtube.com/vi/iTAAUA_fRXw/sddefault.jpg)](https://youtu.be/iTAAUA_fRXw)

[![](http://img.youtube.com/vi/55YOCixB9VI/sddefault.jpg)](https://youtu.be/55YOCixB9VI)

[back to example list](#run-examples)

---

### neck_joint_trajectory_example

首の角度を変更するコード例です。
次のコマンドで頭を上下左右へ向ける動作を実行します。

```
rosrun sciurus17_examples neck_joint_trajectory_example.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_neck_example.gif width = 500px />

#### Videos

[![](http://img.youtube.com/vi/_4J5bpFNQuI/sddefault.jpg)](https://youtu.be/_4J5bpFNQuI)

[![](http://img.youtube.com/vi/scge_3v7-EA/sddefault.jpg)](https://youtu.be/scge_3v7-EA)

[back to example list](#run-examples)

---

### waist_joint_trajectory_example

腰の角度を変更するコード例です。
次のコマンドで腰を左右へひねる動作を実行します。

```
rosrun sciurus17_examples waist_joint_trajectory_example.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_waist_example.gif width = 500px />

### Videos

[![](http://img.youtube.com/vi/sxu-kN4Qc-o/sddefault.jpg)](https://youtu.be/sxu-kN4Qc-o)

[back to example list](#run-examples)

---

### pick_and_place_demo

右手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。腰の回転も使用します。

```
rosrun sciurus17_examples pick_and_place_right_arm_demo.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_pick_and_place_right.gif width = 500px />

左手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_left_arm_demo.py
```
<img src = https://rt-net.github.io/images/sciurus17/gazebo_pick_and_place_left.gif width = 500px />

両手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_two_arm_demo.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_pick_and_place_two.gif width = 500px />

#### Videos

[![](http://img.youtube.com/vi/kjaiWhr-dLg/sddefault.jpg)](https://youtu.be/kjaiWhr-dLg)

[![](http://img.youtube.com/vi/UycaNEHWbv8/sddefault.jpg)](https://youtu.be/UycaNEHWbv8)

[![](http://img.youtube.com/vi/GgKYfSm1NY4/hqdefault.jpg)](https://youtu.be/xo3OiJgu7wg)

[back to example list](#run-examples)

---

### hand_position_publisher

tfの機能でリンク位置を求めるノード例です。  
l_link7とr_link7について、base_linkを基準とした座標をそれぞれ`/sciurus17/hand_pos/left`トピックと
`/sciurus17/hand_pos/right`トピックへ配信します。  
次のコマンドでノードを起動します。  

```
rosrun sciurus17_examples hand_position_publisher_example.py
```

[back to example list](#run-examples)

---

### head_camera_tracking

頭のカメラを使うコード例です。
OpenCVを使ってボール追跡と顔追跡をします。

次のコマンドでノードを起動します。
```sh
rosrun sciurus17_examples head_camera_tracking.py
```

*ボール追跡をする場合*

[`./scripts/head_camera_tracking.py`](./scripts/head_camera_tracking.py)を編集します。

```python
def _image_callback(self, ros_image):
    # ~~~ 省略 ~~~

        # オブジェクト(特定色 or 顔) の検出
        output_image = self._detect_orange_object(input_image)
        # output_image = self._detect_blue_object(input_image)
        # output_image = self._detect_face(input_image)
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_head_camera.gif width = 500px />

*顔追跡をする場合*

[`./scripts/head_camera_tracking.py`](./scripts/head_camera_tracking.py)を編集します。

顔追跡にはカスケード型分類器を使用します。

カスケードファイルのディレクトリを設定してください。
**USER_NAME** は環境に合わせて書き換えてください。

```python
class ObjectTracker:
    def __init__(self):
        # ~~~ 省略 ~~~

        # カスケードファイルの読み込み
        # 例
        # self._face_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_frontalface_alt2.xml")
        # self._eyes_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_eye.xml")
        self._face_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_frontalface_alt2.xml")
        self._eyes_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_eye.xml")
```

```python
def _image_callback(self, ros_image):
    # ~~~ 省略 ~~~

        # オブジェクト(特定色 or 顔) の検出
        # output_image = self._detect_orange_object(input_image)
        # output_image = self._detect_blue_object(input_image)
        output_image = self._detect_face(input_image)
```


#### Videos

[![](http://img.youtube.com/vi/W39aswfINNU/sddefault.jpg)](https://youtu.be/W39aswfINNU)

[![](http://img.youtube.com/vi/I67OD25NkMg/sddefault.jpg)](https://youtu.be/I67OD25NkMg)

動画で使用しているボールは、アールティショップの
[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
で購入できます。

[back to example list](#run-examples)

---

### chest_camera_tracking

胸のカメラを使うコード例です。
OpenCVを使ってボール追跡をします。

次のコマンドでノードを起動します。
```sh
rosrun sciurus17_examples chest_camera_tracking.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_chest_camera.gif width = 500px />

*顔追跡とボール追跡の同時実行*

頭カメラと胸のカメラの両方を使って、顔追跡とボール追跡をします。

```sh
rosrun sciurus17_examples head_camera_tracking.py

# 別のターミナルで実行
rosrun sciurus17_examples chest_camera_tracking.py
```

#### Videos

[![](http://img.youtube.com/vi/wscw-I4wCaM/sddefault.jpg)](https://youtu.be/wscw-I4wCaM)

[![](http://img.youtube.com/vi/c81I0GaC2DU/sddefault.jpg)](https://youtu.be/c81I0GaC2DU)

[back to example list](#run-examples)

---

### depth_camera_tracking

頭の深度カメラを使うコード例です。
指定深度内の物体を追跡します。

次のコマンドでノードを起動します。
```sh
rosrun sciurus17_examples depth_camera_tracking.py
```

デフォルトでは検出範囲を4段階に分けています。
検出範囲を変更する場合は[`./scripts/depth_camera_tracking.py`](./scripts/depth_camera_tracking.py)を編集します。

```python
    def _detect_object(self, input_depth_image):
        # 検出するオブジェクトの大きさを制限する
        MIN_OBJECT_SIZE = 10000 # px * px
        MAX_OBJECT_SIZE = 80000 # px * px

        # 検出範囲を4段階設ける
        # 単位はmm
        DETECTION_DEPTH = [
                (500, 700),
                (600, 800),
                (700, 900),
                (800, 1000)]
```

[back to example list](#run-examples)

---

### preset_pid_gain_example

`sciurus17_control`の`preset_reconfigure`を使うコード例です。
サーボモータのPIDゲインを一斉に変更できます。

プリセットは[sciurus17_control/scripts/preset_reconfigure.py](../sciurus17_control/scripts/preset_reconfigure.py)
にて編集できます。

次のコマンドを実行すると、`preset_reconfigure.py`と`preset_pid_gain_example.py`のノードを起動します。

```sh
roslaunch sciurus17_examples preset_pid_gain_example.launch
```

[back to example list](#run-examples)

---

### box_stacking_example

<img src = https://rt-net.github.io/images/sciurus17/gazebo_box_stacking.gif width = 500px />

[PointCloudLibrary](http://pointclouds.org/)を用いて箱を検出し、箱を積み重ねるコード例です。

次のコマンドでノードを起動します。

```sh
roslaunch sciurus17_examples box_stacking_example.launch
```

RVizで`visualization_msgs/MarkerArray`の`/sciurus17/example/markers`を表示すると検出した箱を確認できます。

<img src = https://rt-net.github.io/images/sciurus17/rviz_box_stacking.png width = 500px />

#### Videos

[![](http://img.youtube.com/vi/vu0prnHfKtU/sddefault.jpg)](https://youtu.be/vu0prnHfKtU)

[back to example list](#run-examples)

---

### current_control_right_arm

右腕を電流制御モードに変更して動かす方法を紹介します。

---

電流制御モードは、位置制御モードと異なり、サーボに設定された角度リミットが**無効**になります。
**ユーザーの責任において十分に安全に注意した上でご使用下さい。**
当該製品および当ソフトウェアの使用中に生じたいかなる損害も株式会社アールティでは一切の責任を負いかねます。

**`Ctrl+c`でサンプルを終了する前に右腕を支えて下さい。
右腕が脱力したときに、腕が物にぶつかる可能性があります。**

---

#### Gazeboで動かす場合

右腕の`hardware_interface`を変更するため、オプションを追加してGazeboを起動します。

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_effort_right_arm:=true
```

#### 実機を動かす場合

実機を動かす前に、
[Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
等のアプリケーションを用いて、
右腕のサーボモータ(ID2 ~ ID8)の`Operating Mode`を位置制御から電流制御に変更します。

※サーボモータの制御モード変更については
[sciurus17_controlのREADME](../sciurus17_control/README.md)
も参照して下さい。

その後、
[sciurus17_control/config/sciurus17_cotrol1.yaml](../sciurus17_control/config/sciurus17_control1.yaml)
を次のように編集します。

- コントローラの種類を`effort_controllers/JointTrajectoryController`に変更。

```diff
right_arm_controller:
-  type: "position_controllers/JointTrajectoryController"
+  type: "effort_controllers/JointTrajectoryController"
  publish_rate: 500
```

- 制御モードを`3(位置制御)`から`0(電流制御)`に変更。

```diff
-    r_arm_joint1: {id: 2, center: 2048, home: 2048, effort_const: 2.79, mode: 3 }
-    r_arm_joint2: {id: 3, center: 2048, home: 1024, effort_const: 2.79, mode: 3 }
-    r_arm_joint3: {id: 4, center: 2048, home: 2048, effort_const: 1.69, mode: 3 }
-    r_arm_joint4: {id: 5, center: 2048, home: 3825, effort_const: 1.79, mode: 3 }
-    r_arm_joint5: {id: 6, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
-    r_arm_joint6: {id: 7, center: 2048, home:  683, effort_const: 1.79, mode: 3 }
-    r_arm_joint7: {id: 8, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
     r_hand_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }

+    r_arm_joint1: {id: 2, center: 2048, home: 2048, effort_const: 2.79, mode: 0 }
+    r_arm_joint2: {id: 3, center: 2048, home: 1024, effort_const: 2.79, mode: 0 }
+    r_arm_joint3: {id: 4, center: 2048, home: 2048, effort_const: 1.69, mode: 0 }
+    r_arm_joint4: {id: 5, center: 2048, home: 3825, effort_const: 1.79, mode: 0 }
+    r_arm_joint5: {id: 6, center: 2048, home: 2048, effort_const: 1.79, mode: 0 }
+    r_arm_joint6: {id: 7, center: 2048, home:  683, effort_const: 1.79, mode: 0 }
+    r_arm_joint7: {id: 8, center: 2048, home: 2048, effort_const: 1.79, mode: 0 }
     r_hand_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
```

ファイル変更後に下記コマンドを実行し、sciurus17のノードを起動します。

```sh
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

コントローラのPIDゲインは、
[sciurus17_control/config/sciurus17_cotrol1.yaml](../sciurus17_control/config/sciurus17_control1.yaml)
で設定されています。
Sciurus17の個体によっては目標姿勢に到達しなかったり、振動する場合があります。
適宜、PIDゲインを変更して下さい。

```yaml
  right_arm_controller:
    type: "effort_controllers/JointTrajectoryController"
    # --- 省略 ---

    # for current control
    gains:
      r_arm_joint1: { p: 5.0,  d: 0.1, i: 0.0 }
      r_arm_joint2: { p: 5.0,  d: 0.1, i: 0.0 }
      r_arm_joint3: { p: 5.0,  d: 0.1, i: 0.0 }
      r_arm_joint4: { p: 5.0,  d: 0.1, i: 0.0 }
      r_arm_joint5: { p: 1.0,  d: 0.1, i: 0.0 }
      r_arm_joint6: { p: 1.0,  d: 0.1, i: 0.0 }
      r_arm_joint7: { p: 1.0,  d: 0.1, i: 0.0 }
```

#### Videos

[![](https://img.youtube.com/vi/NF6cyEOdiuQ/sddefault.jpg)](https://youtu.be/NF6cyEOdiuQ)

[back to example list](#run-examples)

---

### current_control_left_wrist

左手首を電流制御モードに変更して動かす方法を紹介します。

#### Gazeboで動かす場合

左手首の`hardware_interface`を変更するため、オプションを追加してGazeboを起動します。

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_effort_left_wrist:=true
```

#### 実機を動かす場合

実機を動かす前に、
[Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
等のアプリケーションを用いて、
左手首のサーボモータ(ID16)の`Operating Mode`を位置制御から電流制御に変更します。

その後、
[sciurus17_control/config/sciurus17_cotrol2.yaml](../sciurus17_control/config/sciurus17_control2.yaml)
を次のように編集します。

- 手首ジョイントのコントローラを追加

```diff
      goal_time: 0.0
      stopped_velocity_tolerance: 1.0

+  left_wrist_controller:
+    type: "effort_controllers/JointEffortController"
+    joint: l_arm_joint7
+    pid: {p: 1.0, d: 0.0, i: 0.0}

  left_hand_controller:
```

- 制御モードを`3(位置制御)`から`0(電流制御)`に変更。

```diff
    l_arm_joint6: {id: 15, center: 2048, home: 3413, effort_const: 1.79, mode: 3  }
-   l_arm_joint7: {id: 16, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }
    l_hand_joint: {id: 17, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }

    l_arm_joint6: {id: 15, center: 2048, home: 3413, effort_const: 1.79, mode: 3  }
+   l_arm_joint7: {id: 16, center: 2048, home: 2048, effort_const: 1.79, mode: 0  }
    l_hand_joint: {id: 17, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }
```

最後に、
[sciurus17_control/launch/controller2.launch](./sciurus17_control/launch/controller2.launch)
を編集します。

```diff
    <node name="controller_manager"
        pkg="controller_manager"
        type="spawner" respawn="false"
        output="screen"
        args="joint_state_controller
-             left_arm_controller
+             left_wrist_controller
              left_hand_controller"/>
```

ファイル変更後に下記コマンドを実行し、sciurus17のノードを起動します。

```sh
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

#### サンプルの実行

次のコマンドを実行すると、
左手首を±90度に変化させるサンプルプログラムが起動します。

```sh
rosrun sciurus17_examples control_effort_wrist.py
```

#### Videos

[![](http://img.youtube.com/vi/_wJYqQ_5zBw/sddefault.jpg)](https://youtu.be/_wJYqQ_5zBw)

[back to example list](#run-examples)

---
