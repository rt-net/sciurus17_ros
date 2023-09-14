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
        self._image_sub = rospy.Subscriber("/sciurus17/camera/color/image_raw", Image, self._image_callback, queue_size=1)
        self._image_pub = rospy.Publisher("~output_image", Image, queue_size=1)
        self._object_rect = [0,0,0,0]
        self._image_shape = Point()
        self._object_detected = False

        self._CV_MAJOR_VERSION, _, _ = cv2.__version__.split('.')

        # カスケードファイルの読み込み
        # 例
        # self._face_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_frontalface_alt2.xml")
        # self._eyes_cascade = cv2.CascadeClassifier("/home/USER_NAME/.local/lib/python2.7/site-packages/cv2/data/haarcascade_eye.xml")
        self._face_cascade = ""
        self._eyes_cascade = ""


    def _image_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            
        # 画像のwidth, heightを取得
        self._image_shape.x = input_image.shape[1]
        self._image_shape.y = input_image.shape[0]

        # オブジェクト(特定色 or 顔) の検出
        output_image = self._detect_orange_object(input_image)
        # output_image = self._detect_blue_object(input_image)
        # output_image = self._detect_face(input_image)

        self._image_pub.publish(self._bridge.cv2_to_imgmsg(output_image, "bgr8"))


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


    def _detect_color_object(self, bgr_image, lower_color, upper_color):
        # 画像から指定された色の物体を検出する

        MIN_OBJECT_SIZE = 1000 # px * px

        # BGR画像をHSV色空間に変換
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 色を抽出するマスクを生成
        mask = cv2.inRange(hsv, lower_color, upper_color)

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

        self._object_detected = False
        if len(rects) > 0:
            # 最も大きい長方形を抽出
            rect = max(rects, key=(lambda x: x[2] * x[3]))

            # 長方形が小さければ検出判定にしない
            if rect[2] * rect[3] > MIN_OBJECT_SIZE:
                # 抽出した長方形を画像に描画する
                cv2.rectangle(bgr_image, 
                        (rect[0], rect[1]), 
                        (rect[0] + rect[2], rect[1] + rect[3]), 
                        (0, 0, 255), thickness=2)

                self._object_rect = rect
                self._object_detected = True

        return bgr_image


    def _detect_orange_object(self, bgr_image):
        # H: 0 ~ 179 (0 ~ 360°)
        # S: 0 ~ 255 (0 ~ 100%)
        # V: 0 ~ 255 (0 ~ 100%)
        lower_orange = np.array([5,127,127])
        upper_orange = np.array([20,255,255])

        return self._detect_color_object(bgr_image, lower_orange, upper_orange)


    def _detect_blue_object(self, bgr_image):
        # H: 0 ~ 179 (0 ~ 360°)
        # S: 0 ~ 255 (0 ~ 100%)
        # V: 0 ~ 255 (0 ~ 100%)
        lower_blue = np.array([100,127,127])
        upper_blue = np.array([110,255,255])

        return self._detect_color_object(bgr_image, lower_blue, upper_blue)


    def _detect_face(self, bgr_image):
        # 画像から顔(正面)を検出する

        SCALE = 4

        # カスケードファイルがセットされているかを確認
        if self._face_cascade == "" or self._eyes_cascade == "":
            rospy.logerr("cascade file does not set")
            return bgr_image

        # BGR画像をグレー画像に変換
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

        # 処理時間短縮のため画像を縮小
        height, width = gray.shape[:2]
        small_gray = cv2.resize(gray, (width/SCALE, height/SCALE))

        # カスケードファイルを使って顔認識
        small_faces = self._face_cascade.detectMultiScale(small_gray)

        self._object_detected = False
        for small_face in small_faces:
            # 顔の領域を元のサイズに戻す
            face = small_face*SCALE
            
            # グレー画像から顔部分を抽出
            roi_gray = gray[
                    face[1]:face[1]+face[3],
                    face[0]:face[0]+face[2]]

            # 顔の中から目を検知
            eyes = self._eyes_cascade.detectMultiScale(roi_gray)

            # 目を検出したら、対象のrect(座標と大きさ)を記録する
            if len(eyes) > 0:
                cv2.rectangle(bgr_image, 
                        (face[0],face[1]), 
                        (face[0]+face[2], face[1]+face[3]), 
                        (0,0,255),2)

                self._object_rect = face
                self._object_detected = True
                break

        return bgr_image


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
    rospy.init_node("head_camera_tracking")

    neck = NeckYawPitch()
    object_tracker = ObjectTracker()

    main()

