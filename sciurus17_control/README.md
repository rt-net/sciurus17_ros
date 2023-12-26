# sciurus17_control

[English](README.en.md) | [日本語](README.md)

Sciurus17のためのパッケージ、 `sciurus17_ros` のcontrolパッケージです。

## 通信ポートの設定

`sciurus17_control`はSciurus17に内蔵された`SciurusSpineボード`とROS実行環境をUSBケーブルで接続した状態でモータ制御を行います。   
SciurusSpineはセットアップ手順にある`create_udev_rules`の実行後にLinuxに接続すると`/dev/sciurus17spine`として認識されます。

Sciurus17との接続に`/dev/sciurus17spine`以外を使用したい場合は、`sciurus17_control/config`に格納されている各yamlファイルに定義されているパラメータを変更することで設定することができます。

```
dynamixel_port:
  port_name: "/dev/sciurus17spine"
```

各ファイルには次の定義が含まれています。  
`sciurus17_control/config/sciurus17_control1.yaml`　右腕の制御に関する定義ファイルです。  
`sciurus17_control/config/sciurus17_control2.yaml`　左腕の制御に関する定義ファイルです。  
`sciurus17_control/config/sciurus17_control3.yaml`　胴体（腰および首）の制御に関する定義ファイルです。  

## ネームスペースとトピック

`sciurus17_control`は`/sciurus17`をルートとするネームスペースにパラメータやトピックを定義します。   
各jointについて次のトピックを配信します。

current：電流値[mA]   
dxl_position：現在角度[360/4096度]   
temp：温度[度]   

## dynamic_reconfigure

`sciurus17_control`は`dynamic_reconfigure`に対応しています。次のコマンドで`rqt_reconfigure`を起動してアクセスすると各Jointのサーボパラメータを変更することができます。

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](https://rt-net.github.io/images/sciurus17/readme_rqt_reconfigure.png)

各パラメータの詳細についてはROBOTIS公式のXM430およびXM540のサーボマニュアルを参照して下さい。   
- [XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [XM540-W270](http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)

## 制御モード設定

Sciurus17は位置制御モードと電流制御モードに対応しています。基本設定は位置制御モードになっていますが、複数箇所の設定変更を行うことで電流制御モードへ切り替えることが可能です。   
右腕ハンドの設定を位置制御モードから電流制御モードへ変更する手順を紹介します。

1. **サーボの設定変更**   
サーボモータの`Operating Mode`（Address：11）を`3：位置制御モード`から`0：電流制御モード`へ設定変更する

2. **`sciurus17_control`の設定変更**   
`sciurus17_control/config/sciurus17_control1.yaml`のハンド部分について次のように変更する   
```diff
right_hand_controller:
- type: "position_controllers/GripperActionController"
+ type: "effort_controllers/GripperActionController"
  publish_rate: 500
  joint: r_hand_joint
+  gains:
+    r_hand_joint: { p: 1.0, i: 0.01, d: 0.1 }
  action_monitor_rate: 10
  state_publish_rate:  100
  stalled_velocity_threshold: 0.001
  goal_tolerance: 0.05
  stall_timeout: 0.1

dynamixel_port:
  port_name: "/dev/sciurus17spine"
  baud_rate: 3000000
  joints:
    - r_arm_joint1
    - r_arm_joint2
    - r_arm_joint3
    - r_arm_joint4
    - r_arm_joint5
    - r_arm_joint6
    - r_arm_joint7
    - r_hand_joint
  r_arm_joint1: {id: 2, center: 2048, home: 2048, effort_const: 2.79, mode: 3 }
  r_arm_joint2: {id: 3, center: 2048, home: 1024, effort_const: 2.79, mode: 3 }
  r_arm_joint3: {id: 4, center: 2048, home: 2048, effort_const: 1.69, mode: 3 }
  r_arm_joint4: {id: 5, center: 2048, home: 3825, effort_const: 1.79, mode: 3 }
  r_arm_joint5: {id: 6, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
  r_arm_joint6: {id: 7, center: 2048, home:  683, effort_const: 1.79, mode: 3 }
  r_arm_joint7: {id: 8, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
- r_hand_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 3 }
+ r_hand_joint: {id: 9, center: 2048, home: 2048, effort_const: 1.79, mode: 0 }
```

PIDの設定値はSciurus17を制御するROS環境によって特性が異なる可能性があります。   


【電流モードに関する注意】   
電流制御モードは、位置制御モードと異なり、サーボに設定された角度リミットが**無効**になります。   
ユーザー自身で作成されたプログラムに適切な制限動作が備わっていない場合、**本体の損傷や、本体が周囲や作業者に接触、あるいは衝突し、失明や打撲による死亡といった思わぬ重大事故が発生する危険があります。**   
ユーザーの責任において十分に安全に注意した上でご使用下さい。   
当該製品および当ソフトウェアの使用中に生じたいかなる損害も株式会社アールティでは一切の責任を負いかねます。
