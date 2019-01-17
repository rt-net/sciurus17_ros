#!/usr/bin/env python
# coding: utf-8

import rospy
import math

# for ObjectTracker
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point

# for NeckPitch
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint

class ObjectTracker:
    def __init__(self):
        self._bridge = CvBridge()
        self._image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self._image_callback, queue_size=1)
        self._image_pub = rospy.Publisher("output_image", Image, queue_size=1)
        self._object_rect = [0,0,0,0]
        self._image_shape = Point()

    def _image_callback(self, ros_image):
        try:
            input_image = self._bridge.imgmsg_to_cv2(ros_image, "bgr8")
        except CvBridgeError, e:
            print e
            
        output_image = self._process_image(input_image)
        self._image_pub.publish(self._bridge.cv2_to_imgmsg(output_image, "bgr8"))


    def _process_image(self, frame):
        # BGR画像をHSV色空間に変換し、HueとSaturationを取り出す
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
        h = hsv[:, :, 0]
        s = hsv[:, :, 1]

        # 赤色っぽい画素を抽出するマスクを生成
        mask = np.zeros(h.shape, dtype=np.uint8)
        mask[(( h < 20) | (h > 200)) & (s > 128)] = 255

        # マスクから輪郭を抽出
        _, contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # 輪郭をすっぽり収める長方形に変換し、配列に格納
        rects = []
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(rect)

        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            cv2.rectangle(frame, 
                    (rect[0], rect[1]), 
                    (rect[0] + rect[2], rect[1] + rect[3]), 
                    (0, 0, 255), thickness=2)

            self._object_rect = rect

        self._image_shape.x = frame.shape[1]
        self._image_shape.y = frame.shape[0]

        return frame

    def get_object_position(self):
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



    def _cleanup(self):
        cv2.destroyAllWindows()



class NeckPitch(object):
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/neck_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.yaw_angle = 0.0
        self.pitch_angle = 0.0

    def set_angle(self, yaw_angle, pitch_angle):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["neck_yaw_joint", "neck_pitch_joint"]
        yawpoint = JointTrajectoryPoint()
        self.yaw_angle = yaw_angle
        self.pitch_angle = pitch_angle
        yawpoint.positions.append(self.yaw_angle)
        yawpoint.positions.append(self.pitch_angle)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))
        return self.__client.get_result()


def main():
    r = rospy.Rate(60)

    counter = 0
    COUNTER_MAX = 100

    yaw_angle = 0
    pitch_angle = 0
    look_right = True

    THRESH_X = 0.1
    THRESH_Y = 0.2

    while not rospy.is_shutdown():
        object_position = object_tracker.get_object_position()

        if object_position.x > THRESH_X:
            yaw_angle -= 1
        elif object_position.x < -THRESH_X:
            yaw_angle += 1

        if object_position.y > THRESH_Y:
            pitch_angle += 1
        elif object_position.y < -THRESH_Y:
            pitch_angle -= 1

        if yaw_angle >= 90:
            yaw_angle = 90
        elif yaw_angle <= -90:
            yaw_angle = -90

        if pitch_angle >= 80:
            pitch_angle = 80
        elif pitch_angle <= -90:
            pitch_angle = -90


        print object_position 
        neck_pitch.set_angle(math.radians(yaw_angle), math.radians(pitch_angle))


        r.sleep()



if __name__ == '__main__':
    rospy.init_node("head_camera_test")

    neck_pitch = NeckPitch()
    object_tracker = ObjectTracker()

    main()


