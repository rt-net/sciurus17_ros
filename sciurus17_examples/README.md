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

### シミュレータを使う場合

実機無しで動作を確認する場合、制御信号のケーブルを接続しない状態で次のコマンドを実行します。  

```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

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

## サンプルの実行方法

`sciurus17_bringup.launch`を実行している状態で各サンプルを実行することができます。  

### gripper_action_example.pyの実行

両腕のハンドを開閉させるコード例です。   
次のコマンドで26度まで開いて閉じる動作を実行します。

```
rosrun sciurus17_examples gripper_action_example.py
```

動作させると[こちら](https://youtu.be/iTAAUA_fRXw)（[rviz](https://youtu.be/55YOCixB9VI)）のような動きになります。

![gripper_action_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_gripper_example.gif)

---

### neck_joint_trajectory_example.pyの実行 

首の角度を変更するコード例です。
次のコマンドで頭を上下左右へ向ける動作を実行します。

```
rosrun sciurus17_examples neck_joint_trajectory_example.py
```

動作させると[こちら](https://youtu.be/_4J5bpFNQuI)（[rviz](https://youtu.be/scge_3v7-EA)）のような動きになります。

![neck_joint_trajectory_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_neck_example.gif)

---

### waist_joint_trajectory_example.pyの実行

腰の角度を変更するコード例です。
次のコマンドで腰を左右へひねる動作を実行します。

```
rosrun sciurus17_examples waist_joint_trajectory_example.py
```

動作させると[こちら](https://youtu.be/sxu-kN4Qc-o)のような動きになります。

![waist_joint_trajectory_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_waist_example.gif)

---

### Pick & Place デモの実行

右手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。腰の回転も使用します。

```
rosrun sciurus17_examples pick_and_place_right_arm_demo.py
```

動作させると[こちら](https://youtu.be/kjaiWhr-dLg)のような動きになります。

![pick_and_place_right_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_right.gif)

左手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_left_arm_demo.py
```

動作させると[こちら](https://youtu.be/UycaNEHWbv8)のような動きになります。

![pick_and_place_left_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_left.gif)

両手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_two_arm_demo.py
```

動作させると[こちら](https://youtu.be/GgKYfSm1NY4)（[rviz](https://youtu.be/xo3OiJgu7wg)）のような動きになります。

![pick_and_place_two_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_two.gif)

---

### hand_position_publisherの実行

tfの機能でリンク位置を求めるノード例です。  
l_link7とr_link7について、base_linkを基準とした座標をそれぞれ`/sciurus17/hand_pos/left`トピックと
`/sciurus17/hand_pos/right`トピックへ配信します。  
次のコマンドでノードを起動します。  

```
rosrun sciurus17_examples hand_position_publisher_example.py
```

---

### head_camera_tracking.pyの実行

頭のカメラを使うコード例です。
OpenCVを使ってボール追跡と顔追跡をします。

次のコマンドでOpenCVのPythonライブラリをインストールしてください。
```sh
pip2 install opencv-python
```

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

動作させると[こちら](https://youtu.be/W39aswfINNU)のような動きになります。

![head_camera_tracking](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_head_camera.gif)

  - 動画で使用しているボールは、アールティショップの
[こちらのページ](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701)
で購入できます。

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

動作させると[こちら](https://youtu.be/I67OD25NkMg)のような動きになります。

---

### chest_camera_tracking.pyの実行

胸のカメラを使うコード例です。
OpenCVを使ってボール追跡をします。

次のコマンドでOpenCVのPythonライブラリをインストールしてください。
```sh
pip2 install opencv-python
```

次のコマンドでノードを起動します。
```sh
rosrun sciurus17_examples chest_camera_tracking.py
```

動作させると[こちら](https://youtu.be/wscw-I4wCaM)のような動きになります。

![chest_camera_tracking](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_chest_camera.gif)

*顔追跡とボール追跡の同時実行*

頭カメラと胸のカメラの両方を使って、顔追跡とボール追跡をします。

```sh
rosrun sciurus17_examples head_camera_tracking.py

# 別のターミナルで実行
rosrun sciurus17_examples chest_camera_tracking.py
```

動作させると[こちら](https://youtu.be/c81I0GaC2DU)のような動きになります。

---

### depth_camera_tracking.pyの実行

頭の深度カメラを使うコード例です。
指定深度内の物体を追跡します。

次のコマンドでOpenCVのPythonライブラリをインストールしてください。
```sh
pip2 install opencv-python
```

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

---

### preset_pid_gain_example.launchの実行

`sciurus17_control`の`preset_reconfigure`を使うコード例です。
サーボモータのPIDゲインを一斉に変更できます。

プリセットは[sciurus17_control/scripts/preset_reconfigure.py](../sciurus17_control/scripts/preset_reconfigure.py)
にて編集できます。

次のコマンドを実行すると、`preset_reconfigure.py`と`preset_pid_gain_example.py`のノードを起動します。

```sh
roslaunch sciurus17_examples preset_pid_gain_example.launch
```

