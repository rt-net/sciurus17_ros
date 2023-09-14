#!/usr/bin/env python
# coding: utf-8

# Copyright 2019 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy
import math
import sys

# for ObjectTracker
import cv2
import numpy as np
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

# for NeckYawPitch
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    JointTrajectoryControllerState
)
from trajectory_msgs.msg import JointTrajectoryPoint

class ObjectTracker:
    def __init__(self):
        self._bridge = CvBridge()

        # 頭カメラのカラー画像
        self._image_sub = rospy.Subscriber("/sciurus17/camera/color/image_raw", Image, self._image_callback, queue_size=1)
        # 頭カメラの深度画像
        # カラー画像と視界が一致するように補正されている
        self._depth_sub = rospy.Subscriber("/sciurus17/camera/aligned_depth_to_color/image_raw", Image, self._depth_callback, queue_size=1)

        self._image_pub = rospy.Publisher("~output_image", Image, queue_size=1)
        self._median_depth_pub = rospy.Publisher("~output_median_depth", Int32, queue_size=1)

        self._color_image = None
        self._median_depth = 0

        self._object_rect = [0,0,0,0]
        self._image_shape = Point()
        self._object_detected = False

        self._CV_MAJOR_VERSION, _, _ = cv2.__version__.split('.')


    def _image_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        self._color_image = input_image


    def _depth_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
            
        # 画像のwidth, heightを取得
        self._image_shape.x = input_image.shape[1]
        self._image_shape.y = input_image.shape[0]

        output_image = self._detect_object(input_image)
        if output_image is not False:
            self._image_pub.publish(self._bridge.cv2_to_imgmsg(output_image, "bgr8"))

        self._median_depth_pub.publish(self._median_depth)


    def get_object_position(self):
        # 画像中心を0, 0とした座標系におけるオブジェクトの座標を出力
        # オブジェクトの座標は-1.0 ~ 1.0に正規化される

        object_center = Point(
                self._object_rect[0] + self._object_rect[2] * 0.5,
                self._object_rect[1] + self._object_rect[3] * 0.5,
                0)

        # 画像の中心を0, 0とした座標系に変換
        translated_point = Point()
        translated_point.x = object_center.x - self._image_shape.x * 0.5
        translated_point.y = -(object_center.y - self._image_shape.y * 0.5)

        # 正規化
        normalized_point = Point()
        if self._image_shape.x != 0 and self._image_shape.y != 0:
            normalized_point.x = translated_point.x / (self._image_shape.x * 0.5)
            normalized_point.y = translated_point.y / (self._image_shape.y * 0.5)

        return normalized_point


    def object_detected(self):
        return self._object_detected


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
        # 検出したオブジェクトを囲う長方形の色
        RECT_COLOR = [
                (0, 0, 255),
                (0, 255, 0),
                (255, 0, 0),
                (255, 255, 255)]

        # カメラのカラー画像を加工して出力する
        # カラー画像を受け取ってない場合はFalseを返す
        output_image = self._color_image
        if output_image is None:
            return False

        self._object_detected = False
        self._median_depth = 0
        for i, depth in enumerate(DETECTION_DEPTH):
            # depth[0] ~ depth[1]の範囲でオブジェクトを抽出する
            mask = cv2.inRange(input_depth_image, depth[0], depth[1])

            # マスクから輪郭を抽出
            if self._CV_MAJOR_VERSION == '4':
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            else:
                _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            # 輪郭を長方形に変換し、配列に格納
            rects = []
            for contour in contours:
                approx = cv2.convexHull(contour)
                rect = cv2.boundingRect(approx)
                rects.append(rect)

            if len(rects) > 0:
                # 最も大きい長方形を抽出
                rect = max(rects, key=(lambda x: x[2] * x[3]))
            
                rect_size = rect[2] * rect[3]
            
                # 長方形の大きさが指定範囲内であるかチェック
                if rect_size > MIN_OBJECT_SIZE and rect_size < MAX_OBJECT_SIZE:
                    # 抽出した長方形を画像に描画する
                    cv2.rectangle(output_image, 
                            (rect[0], rect[1]), 
                            (rect[0] + rect[2], rect[1] + rect[3]), 
                            RECT_COLOR[i], thickness=2)
            
                    self._object_rect = rect
                    self._object_detected = True

                    # 深度画像から長方形の領域を切り取る
                    object_depth = input_depth_image[
                            rect[1]:rect[1]+rect[3],
                            rect[0]:rect[0]+rect[2]]
                    # 深度の中央値を求める
                    self._median_depth =  int(np.median(object_depth))

                    # 中央値のテキストを出力画像に描画する
                    cv2.putText(output_image, str(self._median_depth), 
                            (rect[0], rect[1]+30),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            1, RECT_COLOR[i], 2)
                    break

        return output_image


class NeckYawPitch(object):
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        self.__client.wait_for_server(rospy.Duration(5.0))
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

        self._state_sub = rospy.Subscriber("/sciurus17/controller3/neck_controller/state", 
                JointTrajectoryControllerState, self._state_callback, queue_size=1)

        self._state_received = False
        self._current_yaw = 0.0 # Degree
        self._current_pitch = 0.0 # Degree


    def _state_callback(self, state):
        # 首の現在角度を取得

        self._state_received = True
        yaw_radian = state.actual.positions[0]
        pitch_radian = state.actual.positions[1]

        self._current_yaw = math.degrees(yaw_radian)
        self._current_pitch = math.degrees(pitch_radian)


    def state_received(self):
        return self._state_received


    def get_current_yaw(self):
        return self._current_yaw


    def get_current_pitch(self):
        return self._current_pitch


    def set_angle(self, yaw_angle, pitch_angle, goal_secs=1.0e-9):
        # 首を指定角度に動かす
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]

        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(yaw_angle)
        yawpoint.positions.append(pitch_angle)
        yawpoint.time_from_start = rospy.Duration(goal_secs)
        goal.trajectory.points.append(yawpoint)

        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))
        return self.__client.get_result()


def hook_shutdown():
    # shutdown時に0度へ戻る
    neck.set_angle(math.radians(0), math.radians(0), 3.0)


def main():
    r = rospy.Rate(60)

    rospy.on_shutdown(hook_shutdown)

    # オブジェクト追跡のしきい値
    # 正規化された座標系(px, px)
    THRESH_X = 0.05
    THRESH_Y = 0.05

    # 首の初期角度 Degree
    INITIAL_YAW_ANGLE = 0
    INITIAL_PITCH_ANGLE = 0

    # 首の制御角度リミット値 Degree
    MAX_YAW_ANGLE   = 120
    MIN_YAW_ANGLE   = -120
    MAX_PITCH_ANGLE = 50
    MIN_PITCH_ANGLE = -70

    # 首の制御量
    # 値が大きいほど首を大きく動かす
    OPERATION_GAIN_X = 5.0
    OPERATION_GAIN_Y = 5.0

    # 初期角度に戻る時の制御角度 Degree
    RESET_OPERATION_ANGLE = 3

    # 現在の首角度を取得する
    # ここで現在の首角度を取得することで、ゆっくり初期角度へ戻る
    while not neck.state_received():
        pass
    yaw_angle = neck.get_current_yaw()
    pitch_angle = neck.get_current_pitch()

    look_object = False
    detection_timestamp = rospy.Time.now()

    while not rospy.is_shutdown():
        # 正規化されたオブジェクトの座標を取得
        object_position = object_tracker.get_object_position()

        if object_tracker.object_detected():
            detection_timestamp = rospy.Time.now()
            look_object = True
        else:
            lost_time = rospy.Time.now() - detection_timestamp
            # 一定時間オブジェクトが見つからない場合は初期角度に戻る
            if lost_time.to_sec() > 1.0:
                look_object = False

        if look_object:
            # オブジェクトが画像中心にくるように首を動かす
            if math.fabs(object_position.x) > THRESH_X:
                yaw_angle += -object_position.x * OPERATION_GAIN_X

            if math.fabs(object_position.y) > THRESH_Y:
                pitch_angle += object_position.y * OPERATION_GAIN_Y

            # 首の制御角度を制限する
            if yaw_angle > MAX_YAW_ANGLE:
                yaw_angle = MAX_YAW_ANGLE
            if yaw_angle < MIN_YAW_ANGLE:
                yaw_angle = MIN_YAW_ANGLE

            if pitch_angle > MAX_PITCH_ANGLE:
                pitch_angle = MAX_PITCH_ANGLE
            if pitch_angle < MIN_PITCH_ANGLE:
                pitch_angle = MIN_PITCH_ANGLE

        else:
            # ゆっくり初期角度へ戻る
            diff_yaw_angle = yaw_angle - INITIAL_YAW_ANGLE
            if math.fabs(diff_yaw_angle) > RESET_OPERATION_ANGLE:
                yaw_angle -= math.copysign(RESET_OPERATION_ANGLE, diff_yaw_angle)
            else:
                yaw_angle = INITIAL_YAW_ANGLE

            diff_pitch_angle = pitch_angle - INITIAL_PITCH_ANGLE
            if math.fabs(diff_pitch_angle) > RESET_OPERATION_ANGLE:
                pitch_angle -= math.copysign(RESET_OPERATION_ANGLE, diff_pitch_angle)
            else:
                pitch_angle = INITIAL_PITCH_ANGLE

        neck.set_angle(math.radians(yaw_angle), math.radians(pitch_angle))

        r.sleep()


if __name__ == '__main__':
    rospy.init_node("depth_camera_tracking")

    neck = NeckYawPitch()
    object_tracker = ObjectTracker()

    main()

