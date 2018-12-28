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

## サンプルの実行方法

`sciurus17_bringup.launch`を実行している状態で各サンプルを実行することができます。  

### gripper_action_example.pyの実行

両腕のハンドを開閉させるコード例です。   
次のコマンドで26度まで開いて閉じる動作を実行します。

```
rosrun sciurus17_examples gripper_action_example.py
```

動作させると[こちら](https://youtu.be/iTAAUA_fRXw)（[rviz](https://youtu.be/55YOCixB9VI)）のような動きになります。

### neck_joint_trajectory_example.pyの実行 

首の角度を変更するコード例です。
次のコマンドで頭を上下左右へ向ける動作を実行します。

```
rosrun sciurus17_examples neck_joint_trajectory_example.py
```

動作させると[こちら](https://youtu.be/_4J5bpFNQuI)（[rviz](https://youtu.be/scge_3v7-EA)）のような動きになります。

### waist_joint_trajectory_example.pyの実行

腰の角度を変更するコード例です。
次のコマンドで腰を左右へひねる動作を実行します。

```
rosrun sciurus17_examples waist_joint_trajectory_example.py
```

動作させると[こちら](https://youtu.be/gfXxxTssO-4)（[rviz](https://youtu.be/l6LbNI9fzMs)）のような動きになります。

### Pick & Place デモの実行

右手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。腰の回転も使用します。

```
rosrun sciurus17_examples pick_and_place_right_arm_demo.py
```

動作させると[こちら](https://youtu.be/kjaiWhr-dLg)のような動きになります。

左手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_left_arm_demo.py
```

動作させると[こちら](https://youtu.be/UycaNEHWbv8)のような動きになります。

両手でターゲットを掴んで動かすデモ動作を次のコマンドで実行します。

```
rosrun sciurus17_examples pick_and_place_two_arm_demo.py
```

動作させると[こちら](https://youtu.be/GgKYfSm1NY4)（[rviz](https://youtu.be/xo3OiJgu7wg)）のような動きになります。

### hand_position_publisherの実行

tfの機能でリンク位置を求めるノード例です。  
l_link7とr_link7について、base_linkを基準とした座標をそれぞれ`/sciurus17/hand_pos/left`トピックと
`/sciurus17/hand_pos/right`トピックへ配信します。  
次のコマンドでノードを起動します。  

```
rosrun sciurus17_examples hand_position_publisher_example.py
```

