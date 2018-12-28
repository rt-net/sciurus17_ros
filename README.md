sciurus17_ros
====
人型上半身17軸＋ハンド2軸のロボット「Sciurus17」のROSパッケージです．   

製品ページはこちらです。  
[https://www.rt-net.jp/products/sciurus17](https://www.rt-net.jp/products/sciurus17)

ROS Wikiはこちらです。  
[https://wiki.ros.org/sciurus17](https://wiki.ros.org/sciurus17)

## 動作環境

以下の環境にて動作確認を行っています。

- OS
  - Ubuntu 16.04.5 LTS
- ROS
  - ROS Kinetic Kame 1.12.14
  - Rviz 1.12.16
  - MoveIt! 0.9.12
## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/ja/kinetic/Installation/Ubuntu)を参照しROSをインストールします。

- [Intel RealeSense SDK 2.0](https://github.com/IntelRealSense/librealsense)をインストールします。
  - [公式ページの手順](https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md)に
  従って、SDK(librealsense2-*)をインストールしてください。

- ROSパッケージ[realsense2_camera](http://wiki.ros.org/realsense2_camera)をダウンロード&ビルドします。
  ```bash
  cd ~/catkin_ws/src/
  git clone https://github.com/intel-ros/realsense
  cd ~/catkin_ws
  catkin_make
  ```

- 本パッケージをダウンロードします。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_ros.git
  ```

- 依存関係にあるパッケージをインストールします。

  ```bash
  cd ~/catkin_ws/src
  rosdep install -r -y --from-paths . --ignore-src
  ```

- `catkin_make`を使用して本パッケージをビルドします。

  ```bash
  cd ~/catkin_ws && catkin_make
  source ~/catkin_ws/devel/setup.bash
  ```

### `apt`を使用してインストールする方法

後日提供予定です。

## セットアップ方法

次の方法で`sciurus17_control`が実機と通信するために用いるUSBシリアル変換デバイス名を固定します。

```bash
roscd sciurus17_tools/scripts/
./create_udev_rules
```

実行後に再起動しSciurus17を接続すると`/dev/sciurus17spine`が作成されるようになります。

## パッケージ概要

Sciurus17の各パッケージはsciurus17_rosにまとめています。  

### sciurus17_control
Sciurus17の制御を行うパッケージです。  
dynamixel_sdkのC++ライブラリが必要です。  
実機との通信には`/dev/sciurus17spine`へのアクセス権が必要です。

通信に使用するポートの名前やサーボ情報は次の設定ファイルに記載します。  

- `config/sciurus17_control1.yaml`
- `config/sciurus17_control2.yaml`
- `config/sciurus17_control3.yaml`

設定されたUSBポートが無い場合、コントローラからの指示通りの値を返すダミージョイントモードで動作します。  
機能制限がありますがハードウェアを使用しなくてもデバッグが出来るので便利に使って下さい。  

起動時は設定されたホームポジションへ5秒かけて移動します。  
ノードを停止するとサーボをブレーキモードに変更してから終了するので安全に停止することができます。  

### sciurus17_description

Sciurus17のモデルデータやリンクとジョイントの構成を定義するパッケージです。
MoveIt!やGazeboから呼び出されます。

### sciurus17_moveit_config

MoveIt!のパッケージです。下記のコマンドで起動します。

```bash
roslaunch sciurus17_moveit_config demo.launch
```

### sciurus17_msgs

Sciurus17で使用する独自メッセージを定義するパッケージです。

### sciurus17_vision

カメラのlaunchファイルや画像認識を行うノードを定義するパッケージです。

### sciurus17_bringup

Sciurus17の起動に必要なlaunchファイルをまとめたパッケージです。

### sciurus17_tools

Sciurus17を活用するためのオプションツールをまとめたパッケージです。
頭部カメラを使用する場合、先に次のコマンドでカメラのリセットを行うと動作が安定しやすくなります。

```bash
rosrun sciurus17_tools realsense_hwreset
```

### sciurus17_examples

Sciurus17を動作させるためのサンプルコードをまとめたパッケージです。
