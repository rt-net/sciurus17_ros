[English](README.en.md) | [日本語](README.md)

# sciurus17_examples

This package includes examples to control Sciurus17 using `sciurus17_ros`.

## How to launch Sciurus17 base packages

1. Connect cables of a head camera, a chest camera and a control board to a PC.
1. Power on the Sciurus17 and the camera device names are shown in the `/dev` directory.
1. Open terminal and launch `sciurus17_bringup.launch` of `sciurus17_bringup` package.

This launch file has arguments:

- use_rviz (default: true)  
- use_head_camera (default: true)  
- use_chest_camera (default: true)  

### Using virtual Sciurus17

To launch Sciurus17 base packages **without** Sciurus17 hardware, 
unplug the control board's cable from the PC,
then launch nodes with the following command:

```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

### Using real Sciurus17

Launch the base packages with the following command:

```
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

### Using without cameras

Launch the base packages with arguments:

```
roslaunch sciurus17_bringup sciurus17_bringup.launch use_head_camera:=false use_chest_camera:=false
```

### Using without RViz

To reduce the CPU load of the PC, launch the base packages with arguments:

```
roslaunch sciurus17_bringup sciurus17_bringup.launch use_rviz:=false
```

### Using Gazebo simulator

Launch the packages with the following command:

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch

# without RViz
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_rviz:=false
```

## Run Examples

Following examples will be executable after launch Sciurus17 base packages.

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

This is an example to open/close the grippers of the two arms.

Run a node with the following command:

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

This is an example to change angles of the neck.

Run a node with the following command:

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

This is an example to change angles of the waist.

Run a node with the following command:

```
rosrun sciurus17_examples waist_joint_trajectory_example.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_waist_example.gif width = 500px />

### Videos

[![](http://img.youtube.com/vi/sxu-kN4Qc-o/sddefault.jpg)](https://youtu.be/sxu-kN4Qc-o)

[back to example list](#run-examples)

---

### pick_and_place_demo

This is an example to pick and place a small object with right hand while turning the waist.

Run a node with the following command:

```
rosrun sciurus17_examples pick_and_place_right_arm_demo.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_pick_and_place_right.gif width = 500px />

This is an example to pick and place a small object with left hand.

Run a node with the following command:

```
rosrun sciurus17_examples pick_and_place_left_arm_demo.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_pick_and_place_left.gif width = 500px />

This is an example to pick and place a small object with both hands.

Run a node with the following command:

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

This is an example to receive link positions from `tf` server.

This example receives transformed positions `l_link7` and `r_link7` based on `base_link`
from `tf` server, then publishes these positions as topics named 
`/sciurus17/hand_pos/left` and `/sciurus17/hand_pos/right`.

Run a node with the following command:

```
rosrun sciurus17_examples hand_position_publisher_example.py
```

[back to example list](#run-examples)

---

### head_camera_tracking

This is an example to use the head camera images and OpenCV library for ball tracking and face tracking.

Run a node with the following command:

```sh
rosrun sciurus17_examples head_camera_tracking.py
```

*For ball tracking*

Edit [`./scripts/head_camera_tracking.py`](./scripts/head_camera_tracking.py) as follows:

```python
def _image_callback(self, ros_image):
    # ...

        # Detect an object (specific color or face)
        output_image = self._detect_orange_object(input_image)
        # output_image = self._detect_blue_object(input_image)
        # output_image = self._detect_face(input_image)
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_head_camera.gif width = 500px />

*For face tracking*

Edit [`./scripts/head_camera_tracking.py`](./scripts/head_camera_tracking.py) as follows:

This example uses Cascade Classifier for face tracking.

Please edit the directories of cascade files in the script file.
**USER_NAME**  depends on user environments.

```python
class ObjectTracker:
    def __init__(self):
        # ...

        # Load cascade files
        # Example:
        # self._face_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_frontalface_alt2.xml")
        # self._eyes_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_eye.xml")
        self._face_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_frontalface_alt2.xml")
        self._eyes_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_eye.xml")
```

```python
def _image_callback(self, ros_image):
    # ...

        # Detect an object (specific color or face)
        # output_image = self._detect_orange_object(input_image)
        # output_image = self._detect_blue_object(input_image)
        output_image = self._detect_face(input_image)
```

#### Videos

[![](http://img.youtube.com/vi/W39aswfINNU/sddefault.jpg)](https://youtu.be/W39aswfINNU)

[![](http://img.youtube.com/vi/I67OD25NkMg/sddefault.jpg)](https://youtu.be/I67OD25NkMg)

This orange ball can be purchased from
[this page](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
in RT ROBOT SHOP.

[back to example list](#run-examples)

---

### chest_camera_tracking

This is an example to use the chest camera images and OpenCV library for ball tracking.

Run a node with the following command:

```sh
rosrun sciurus17_examples chest_camera_tracking.py
```

<img src = https://rt-net.github.io/images/sciurus17/gazebo_chest_camera.gif width = 500px />

*Execute face tracking and ball tracking simultaneously*

Launch nodes with the following commands for face tracking with the head camera and 
for ball tracking with the chest camera.

```sh
rosrun sciurus17_examples head_camera_tracking.py

# Open another terminal
rosrun sciurus17_examples chest_camera_tracking.py
```

#### Videos

[![](http://img.youtube.com/vi/wscw-I4wCaM/sddefault.jpg)](https://youtu.be/wscw-I4wCaM)

[![](http://img.youtube.com/vi/c81I0GaC2DU/sddefault.jpg)](https://youtu.be/c81I0GaC2DU)

[back to example list](#run-examples)

---

### depth_camera_tracking

This is an example to use the depth camera on the head for object tracking.

Run a node with the following command:

```sh
rosrun sciurus17_examples depth_camera_tracking.py
```

The default detection range is separated into four stages.

To change the detection range, edit [`./scripts/depth_camera_tracking.py`](./scripts/depth_camera_tracking.py) as the followings:

```python
    def _detect_object(self, input_depth_image):
        # Limitation of object size
        MIN_OBJECT_SIZE = 10000 # px * px
        MAX_OBJECT_SIZE = 80000 # px * px

        # The detection range is separated into four stages.
        # Unit: mm
        DETECTION_DEPTH = [
                (500, 700),
                (600, 800),
                (700, 900),
                (800, 1000)]
```

[back to example list](#run-examples)

---

### preset_pid_gain_example

This is an example to change PID gains of the servo motors in bulk 
using `preset_reconfigure` of `sciurus17_control`.

Lists of PID gain preset values can be edited in
[sciurus17_control/scripts/preset_reconfigure.py](../sciurus17_control/scripts/preset_reconfigure.py).

Launch nodes `preset_reconfigure.py` and `preset_pid_gain_example.py` with the following command:

```sh
roslaunch sciurus17_examples preset_pid_gain_example.launch
```

[back to example list](#run-examples)

---

### box_stacking_example

<img src = https://rt-net.github.io/images/sciurus17/gazebo_box_stacking.gif width = 500px />

This is an example to detect boxes via [PointCloudLibrary](http://pointclouds.org/)
and stack the boxes.

Launch nodes with the following command:

```sh
roslaunch sciurus17_examples box_stacking_example.launch
```

To visualize the result of box detection, please add `/sciurus17/example/markers` of `visualization_msgs/MarkerArray` in Rviz.

<img src = https://rt-net.github.io/images/sciurus17/rviz_box_stacking.png width = 500px />

#### Videos

[![](http://img.youtube.com/vi/vu0prnHfKtU/sddefault.jpg)](https://youtu.be/vu0prnHfKtU)

[back to example list](#run-examples)

### current_control_right_arm

This section shows how to change the right arm to current-controlled mode and move it.

---

Unlike the position control mode, the angle limit set on the servo becomes **invalid** in the current control mode.  
RT Corporation assumes no responsibility for any damage that may occur during use of the product or this software.

**Please hold onto the robot's right arm before exiting the sample with `Ctrl+c`.
When the sample ends, the right arm will deactivate and become free, which may lead to a clash.**

---

#### Using Gazebo simulator for current_control_right_arm

Start Gazebo with additional options to change the `hardware_interface` of the right arm.

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_effort_right_arm:=true
```

#### Using real Sciurus17 for current_control_right_arm

Before running the real Sciurus17,
change the `Operating Mode` of the servo motors (ID2 ~ ID8) of the right arm
from position control to current control using an application such as [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

※Please also refer to the [README of sciurus17_control](../sciurus17_control/README.md)
for information on changing the control mode of the servo motor.

Then edit
[sciurus17_control/config/sciurus17_cotrol1.yaml](../sciurus17_control/config/sciurus17_control1.yaml)
as follows.

- Change the controller type to `effect_controllers/JointTrajectoryController`.

```diff
right_arm_controller:
-  type: "position_controllers/JointTrajectoryController"
+  type: "effort_controllers/JointTrajectoryController"
  publish_rate: 500
```

- Change control mode from `3 (position control)` to `0 (current control)`.

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

After changing the file, run the following command to start the sciurus17 node.

```sh
roslaunch sciurus17_bringup sciurus17_bringup.launch
```
The PID gain of the controller is set in
[sciurus17_control/config/sciurus17_cotrol1.yaml](../sciurus17_control/config/sciurus17_control1.yaml).

Depending on the individual Sciurus17, it may not reach the target attitude or may vibrate. Change the PID gain accordingly.

```yaml
  right_arm_controller:
    type: "effort_controllers/JointTrajectoryController"

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

This section shows how to change the left wrist to current-controlled mode and move it.

#### Using Gazebo simulator for current_control_left_wrist

Start Gazebo with additional options to change the `hardware_interface` of the left wrist.

```sh
roslaunch sciurus17_gazebo sciurus17_with_table.launch use_effort_left_wrist:=true
```

#### Using real Sciurus17 for current_control_left_wrist

Before running the real Sciurus17,
change the `Operating Mode` of the servo motor (ID16) of the left wrist
from position control to current control using an application such as [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).


Then edit
[sciurus17_control/config/sciurus17_cotrol2.yaml](../sciurus17_control/config/sciurus17_control2.yaml)
as follows.

- Add a controller for wrist joints.

```diff
      goal_time: 0.0
      stopped_velocity_tolerance: 1.0

+  left_wrist_controller:
+    type: "effort_controllers/JointEffortController"
+    joint: l_arm_joint7
+    pid: {p: 1.0, d: 0.0, i: 0.0}

  left_hand_controller:
```

- Change control mode from `3 (position control)` to `0 (current control)`.

```diff
    l_arm_joint6: {id: 15, center: 2048, home: 3413, effort_const: 1.79, mode: 3  }
-   l_arm_joint7: {id: 16, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }
    l_hand_joint: {id: 17, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }

    l_arm_joint6: {id: 15, center: 2048, home: 3413, effort_const: 1.79, mode: 3  }
+   l_arm_joint7: {id: 16, center: 2048, home: 2048, effort_const: 1.79, mode: 0  }
    l_hand_joint: {id: 17, center: 2048, home: 2048, effort_const: 1.79, mode: 3  }
```
Finally, edit
[sciurus17_control/launch/controller2.launch](./sciurus17_control/launch/controller2.launch).

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

After changing the file, run the following command to start the sciurus17 node.

```sh
roslaunch sciurus17_bringup sciurus17_bringup.launch
```

#### Run the example

Run the following command to start a example for moving the left wrist in a range of ±90 degrees.

```sh
rosrun sciurus17_examples control_effort_wrist.py
```

#### Videos

[![](http://img.youtube.com/vi/_wJYqQ_5zBw/sddefault.jpg)](https://youtu.be/_wJYqQ_5zBw)

[back to example list](#run-examples)

---
