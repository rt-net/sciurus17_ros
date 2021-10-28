[English](README.en.md) | [日本語](README.md)

sciurus17_ros
====

[![industrial_ci](https://github.com/rt-net/sciurus17_ros/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/sciurus17_ros/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

![sciurus17_gazebo](https://rt-net.github.io/images/sciurus17/sciurus17_gazebo.png "sciurus17_gazebo")

人型上半身17軸＋ハンド2軸のロボット「Sciurus17」のROSパッケージです．   

製品ページはこちらです。  
[https://www.rt-net.jp/products/sciurus17](https://www.rt-net.jp/products/sciurus17)

ROS Wikiはこちらです。  
[https://wiki.ros.org/sciurus17](https://wiki.ros.org/sciurus17)

ROSのサンプルコード集はこちらです。  
[sciurus17_examples](https://github.com/rt-net/sciurus17_ros/tree/master/sciurus17_examples)

## 動作環境

以下の環境にて動作確認を行っています。

- ROS Melodic
  - OS: Ubuntu 18.04.3 LTS
  - ROS Distribution: Melodic Morenia 1.14.3
  - Rviz 1.12.16
  - MoveIt! 1.13.3
  - Gazebo 9.0.0
  
## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/ja/noetic/Installation/Ubuntu)を参照しROSをインストールします。

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

※画像処理を行うサンプルを動かすためには
[sciurus17_examples/README.md](./sciurus17_examples/README.md)
に従ってOpenCVをインストールして下さい。

### 通信機器のセットアップ

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

### sciurus17_gazebo

GazeboでSciurus17のシミュレーションを行うパッケージです。  
次のコマンドで起動します。実機との接続やsciurus17_bringupの実行は必要ありません。  
初回起動時のみモデルのローディングに時間がかかります。Gazeboの画面が表示されるまで十分待って下さい。  

```bash
roslaunch sciurus17_gazebo sciurus17_with_table.launch
```

### sciurus17_examples

Sciurus17を動作させるためのサンプルコードをまとめたパッケージです。  
`sciurus17_examples` の使い方については[./sciurus17_examples/README.md](./sciurus17_examples/README.md)を参照してください。  

### 知的財産権について

Sciurus17は、アールティが開発した研究用上半身ロボットです。
このリポジトリのデータ等に関するライセンスについては、LICENSEファイルをご参照ください。
企業による使用については、自社内において研究開発をする目的に限り、本データの使用を許諾します。 
本データを使って自作されたい方は、義務ではありませんが弊社ロボットショップで部品をお買い求めいただければ、励みになります。
商業目的をもって本データを使用する場合は、商業用使用許諾の条件等について弊社までお問合せください。

サーボモータのXM540やXM430に関するCADモデルの使用については、ROBOTIS社より使用許諾を受けています。 
Sciurus17に使用されているROBOTIS社の部品類にかかる著作権、商標権、その他の知的財産権は、ROBOTIS社に帰属します。

### Proprietary Rights

Sciurus17 is an upper body robot developed by RT Corporation for research purposes. Please read the license information contained in this repository to find out more about licensing. Companies are permitted to use Sciurus17 and the materials made available here for internal, research and development purposes only. If you are interested in building your own robot for your personal use by utilizing the information made available here, take your time to visit our website and purchase relevant components and parts – that will certainly help us keep going! Otherwise, if you are interested in manufacturing and commercializing products based on the information herein, please contact us to arrange a license and collaboration agreement with us. 

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors XM540 and XM430. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS. 

