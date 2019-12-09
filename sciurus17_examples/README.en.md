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

### gripper_action_example.py

This is an example to open/close the grippers of the two arms.

Run a node with the following command:

```
rosrun sciurus17_examples gripper_action_example.py
```

Demo Video is [here ](https://youtu.be/iTAAUA_fRXw)([rviz](https://youtu.be/55YOCixB9VI)).

![gripper_action_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_gripper_example.gif)

---

### neck_joint_trajectory_example.py

This is an example to change angles of the neck.

Run a node with the following command:

```
rosrun sciurus17_examples neck_joint_trajectory_example.py
```

Demo Video is [here](https://youtu.be/_4J5bpFNQuI)([rviz](https://youtu.be/scge_3v7-EA)).

![neck_joint_trajectory_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_neck_example.gif)

---

### waist_joint_trajectory_example.py

This is an example to change angles of the waist.

Run a node with the following command:

```
rosrun sciurus17_examples waist_joint_trajectory_example.py
```

Demo Video is [here](https://youtu.be/sxu-kN4Qc-o).

![waist_joint_trajectory_example](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_waist_example.gif)

---

### Pick and Place

This is an example to pick and place a small object with right hand while turning the waist.

Run a node with the following command:

```
rosrun sciurus17_examples pick_and_place_right_arm_demo.py
```

Demo Video is [here](https://youtu.be/kjaiWhr-dLg).

![pick_and_place_right_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_right.gif)

This is an example to pick and place a small object with left hand.

Run a node with the following command:

```
rosrun sciurus17_examples pick_and_place_left_arm_demo.py
```

Demo Video is [here](https://youtu.be/UycaNEHWbv8).

![pick_and_place_left_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_left.gif)

This is an example to pick and place a small object with both hands.

Run a node with the following command:

```
rosrun sciurus17_examples pick_and_place_two_arm_demo.py
```

Demo Video is [here](https://youtu.be/GgKYfSm1NY4)([rviz](https://youtu.be/xo3OiJgu7wg)).

![pick_and_place_two_arm](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_pick_and_place_two.gif)

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

---

### head_camera_tracking.py

This is an example to use the head camera images and OpenCV library for ball tracking and face tracking.

Firstly, install the OpenCV library of Python with the following command:

```sh
pip2 install opencv-python
```

Then, run a node with the following command:

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

Demo Video is [here](https://youtu.be/W39aswfINNU).

![head_camera_tracking](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_head_camera.gif)

  - This orange ball can be purchased from
[this page](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1299_1307&products_id=3701&language=en)
in RT ROBOT SHOP.

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

Demo Video is [here](https://youtu.be/I67OD25NkMg).

---

### chest_camera_tracking.py

This is an example to use the chest camera images and OpenCV library for ball tracking.

Firstly, install the OpenCV library of Python with the following command:

```sh
pip2 install opencv-python
```

Then, run a node with the following command:

```sh
rosrun sciurus17_examples chest_camera_tracking.py
```

Demo Video is [here](https://youtu.be/wscw-I4wCaM).

![chest_camera_tracking](https://github.com/rt-net/sciurus17_ros/blob/images/images/gazebo_chest_camera.gif)

*Execute face tracking and ball tracking simultaneously*

Launch nodes with the following commands for face tracking with the head camera and 
for ball tracking with the chest camera.

```sh
rosrun sciurus17_examples head_camera_tracking.py

# Open another terminal
rosrun sciurus17_examples chest_camera_tracking.py
```

Demo Video is [here](https://youtu.be/c81I0GaC2DU).

---

### depth_camera_tracking.py

This is an example to use the depth camera on the head for object tracking.

Firstly, install the OpenCV library of Python with the following command:

```sh
pip2 install opencv-python
```

Then, run a node with the following command:

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

---

### preset_pid_gain_example.launch

This is an example to change PID gains of the servo motors in bulk 
using `preset_reconfigure` of `sciurus17_control`.

Lists of PID gain preset values can be edited in
[sciurus17_control/scripts/preset_reconfigure.py](../sciurus17_control/scripts/preset_reconfigure.py).

Launch nodes `preset_reconfigure.py` and `preset_pid_gain_example.py` with the following command:

```sh
roslaunch sciurus17_examples preset_pid_gain_example.launch
```

