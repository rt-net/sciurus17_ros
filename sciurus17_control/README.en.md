# sciurus17_control

[English](README.en.md) | [日本語](README.md)

The control package for Sciurus17.

## Communication port settings

`sciurus17_control` communicates with the `SciurusSpine` board built into Sciurus17 via USB for motor control.

SciurusSpine is recognized as `/dev/sciurus17spine` when connected to Linux after execution of `create_udev_rules` in the setup procedure.

If you want to use a device name other than `/dev/sciurus17spine` for connection to Sciurus17,
you need to change the parameters defined in each yaml file in `sciurus17_control/config`.

```yaml
dynamixel_port:
  port_name: "/dev/sciurus17spine"
```

Each file contains the following definitions.  

- `sciurus17_control/config/sciurus17_control1.yaml` defines parameters for right arm control.
- `sciurus17_control/config/sciurus17_control2.yaml` defines parameters for left arm control.
- `sciurus17_control/config/sciurus17_control3.yaml` defines parameters for torso (waist and neck) control.

## Namespaces and topics

`sciurus17_control` defines parameters and topics in a namespace whose root is `/sciurus17`.

The node publishes the following topics for each joint.

- current: current value [mA].
- dxl_position: current angle [360/4096 degrees].
- temp: temperature [degree].

## Dynamic reconfigure

`sciurus17_control` can change the servo parameters of each Joint with `dynamic_reconfigure`.

Start `rqt_reconfigure` with the following command.

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

![rqt_reconfigure](https://rt-net.github.io/images/sciurus17/readme_rqt_reconfigure.png)

Please refer to the official ROBOTIS XM430 and XM540 servo manuals for details on each parameter.

- [XM430-W350](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
- [XM540-W270](http://emanual.robotis.com/docs/en/dxl/x/xm540-w270/)

## Control mode settings

Sciurus17 supports position control mode and current control mode.
The default setting is position control mode, and it can be switched to current control mode by changing the settings in several places.

The following is the procedure to change the setting of the right hand from position control mode to current control mode.

1.**Change the servo setting**.

Change `Operating Mode` (Address: 11) of the servo motor from `3: Position Control Mode` to `0: Current Control Mode`

2.**Change the setting of `sciurus17_control`**.

Change the hand part of `sciurus17_control/config/sciurus17_control1.yaml` as follows

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

The PID setting values may have different characteristics depending on the ROS environment in which Sciurus17 is controlled.  

## Caution on current mode

Unlike the position control mode, the angle limit set on the servo becomes **invalid** in the current control mode.  
If the user's own program is not equipped with the appropriate limit action, there is a risk of serious accidents such as **damage to the main unit, contact or collision of the main unit with surroundings or the operator, loss of eyesight, or death due to bruising.**  
The user is responsible for the usage of this product, and they must exercise due care for safety.  
RT Corporation has no responsibility for any damage that occured during  the usage of the product or this software.
