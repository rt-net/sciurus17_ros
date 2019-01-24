#!/usr/bin/env python
# coding: utf-8

import rospy
import math
import time
import sys

# for ObjectTracker
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

# for WaistYaw
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
        self._image_sub = rospy.Subscriber("/chest_camera_node/image", Image, self._image_callback, queue_size=1)
        self._image_pub = rospy.Publisher("~output_image", Image, queue_size=1)
        self._object_rect = [0,0,0,0]
        self._image_shape = Point()
        self._object_detected = False


    def _image_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            rospy.logerr(e)
            
        # 画像のwidth, heightを取得
        self._image_shape.x = input_image.shape[1]
        self._image_shape.y = input_image.shape[0]

        # オブジェクト(オレンジ色) の検出
        output_image = self._detect_orange_object(input_image)

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


    def _detect_orange_object(self, bgr_image):
        # 画像からオレンジ色の物体を検出する

        MIN_OBJECT_SIZE = 7000 # px * px

        # BGR画像をHSV色空間に変換
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # オレンジ色を抽出するマスクを生成
        # H: 0 ~ 179 (0 ~ 360°)
        # S: 0 ~ 255 (0 ~ 100%)
        # V: 0 ~ 255 (0 ~ 100%)
        lower_orange = np.array([5,127,127])
        upper_orange = np.array([20,255,255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # マスクから輪郭を抽出
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


class WaistYaw(object):
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/waist_yaw_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)

        self._state_sub = rospy.Subscriber("/sciurus17/controller3/waist_yaw_controller/state", 
                JointTrajectoryControllerState, self._state_callback, queue_size=1)

        self._state_received = False
        self._current_yaw = 0.0 # degree

    def _state_callback(self, state):
        # 腰の現在角度を取得

        self._state_received = True

        yaw_radian = state.actual.positions[0]

        self._current_yaw = math.degrees(yaw_radian)

        print self._current_yaw

    
    def state_received(self):
        return self._state_received


    def get_current_yaw(self):
        return self._current_yaw


    # 目標角度の設定と実行
    def set_angle(self, yaw_angle):
        # 腰を指定角度に動かす
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["waist_yaw_joint"]
        yawpoint = JointTrajectoryPoint()
        yawpoint.positions.append(yaw_angle)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))
        return self.__client.get_result()


def main():
    r = rospy.Rate(60)

    # オブジェクト追跡のしきい値
    # 正規化された座標系(px, px)
    THRESH_X = 0.05

    # 腰の制御角度リミット値
    MAX_YAW_ANGLE   = 90
    MIN_YAW_ANGLE   = -90

    # 腰の制御量
    # 値が大きいほど大きく動く
    OPERATION_GAIN_X = 3.0

    # 正面に戻る時の制御角度
    RESET_OPERATION_ANGLE = 3

    # 現在の腰角度を取得する
    # ここで現在の腰角度を取得することで、ゆっくり正面を向くことができる
    while not waist_yaw.state_received():
        pass
    yaw_angle = waist_yaw.get_current_yaw()

    look_object = False
    detection_timestamp = time.time()

    while not rospy.is_shutdown():
        # 正規化されたオブジェクトの座標を取得
        object_position = object_tracker.get_object_position()

        if object_tracker.object_detected():
            detection_timestamp = time.time()
            look_object = True
        else:
            lost_time = time.time() - detection_timestamp
            # 一定時間オブジェクトが見つからない場合は正面を向く
            if lost_time > 1.0:
                look_object = False

        if look_object:
            # オブジェクトが画像中心にくるように首を動かす
            if math.fabs(object_position.x) > THRESH_X:
                yaw_angle += -object_position.x * OPERATION_GAIN_X

            # 腰の角度を制限する
            if math.fabs(yaw_angle) >= MAX_YAW_ANGLE:
                yaw_angle = math.copysign(MAX_YAW_ANGLE, yaw_angle)
        else:
            # ゆっくり正面を向く
            if math.fabs(yaw_angle) > RESET_OPERATION_ANGLE:
                yaw_angle -= math.copysign(RESET_OPERATION_ANGLE, yaw_angle)
            else:
                yaw_angle = 0

        waist_yaw.set_angle(math.radians(yaw_angle))

        r.sleep()


if __name__ == '__main__':
    rospy.init_node("chest_camera_test")

    waist_yaw = WaistYaw()
    object_tracker = ObjectTracker()

    main()
