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
  - ROS Distribution: Melodic Morenia 1.14.9
  - Rviz 1.13.19
  - MoveIt 1.0.8
  - Gazebo 9.0.0
- ROS Noetic
  - OS: Ubuntu 20.04.3 LTS
  - ROS Distribution: Noetic Ninjemys 1.15.8
  - Rviz 1.14.10
  - MoveIt 1.1.5
  - Gazebo 11.5.1
  
## インストール方法

### ソースからビルドする方法

- [ROS Wiki](http://wiki.ros.org/ja/noetic/Installation/Ubuntu)を参照しROSをインストールします。

- 本パッケージをダウンロードします。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_ros.git
  ```

- [sciurus17_description](https://github.com/rt-net/sciurus17_description)パッケージをダウンロードします。
このパッケージには株式会社アールティの[非商用ライセンス](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE)が適用されています。

  ```bash
  cd ~/catkin_ws/src
  git clone https://github.com/rt-net/sciurus17_description.git
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

### v1.0.0以前のバージョンからv2.x.xへ更新する場合

バージョンの違いについては
https://github.com/rt-net/sciurus17_ros/issues/134
を参照してください。

次の手順でパッケージを更新してください。

```bash
# sciurus17_rosを更新
cd ~/catkin_ws/src/sciurus17_ros
git pull origin master

# sciurus17_descriptionをダウンロード
cd ~/catkin_ws/src
git clone https://github.com/rt-net/sciurus17_description.git
rosdep install -r -y --from-paths . --ignore-src

# ビルド環境を初期化し、パッケージを再ビルド
# 同じワークスペースにある、Sciurus17以外の他のROSパッケージについても再ビルドを行います
cd ~/catkin_ws
rm -r build devel
catkin_make
```

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

### sciurus17_moveit_config

MoveItのパッケージです。下記のコマンドで起動します。

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

## ライセンス

(C) 2018 RT Corporation \<support@rt-net.jp\>

各ファイルはライセンスがファイル中に明記されている場合、そのライセンスに従います。
特に明記されていない場合は、Apache License, Version 2.0に基づき公開されています。  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[sciurus17_description](https://github.com/rt-net/sciurus17_description)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[sciurus17_description/LICENSE](https://github.com/rt-net/sciurus17_description/blob/main/LICENSE)を参照してください。

## 開発について

- 本ソフトウェアはオープンソースですが、開発はオープンではありません。
- 本ソフトウェアは基本的にオープンソースソフトウェアとして「AS IS」（現状有姿のまま）で提供しています。
- 本ソフトウェアに関する無償サポートはありません。
- バグの修正や誤字脱字の修正に関するリクエストは常に受け付けていますが、
それ以外の機能追加等のリクエストについては社内のガイドラインを優先します。
詳しくは[コントリビューションガイドライン](./CONTRIBUTING.md)に従ってください。
