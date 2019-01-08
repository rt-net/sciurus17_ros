#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal
)
from trajectory_msgs.msg import JointTrajectoryPoint
import sys
import math

class WaistYaw(object):
    def __init__(self):
        self.__client = actionlib.SimpleActionClient("/sciurus17/controller3/waist_yaw_controller/follow_joint_trajectory",
                                                     FollowJointTrajectoryAction)
        if not self.__client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.yaw_angle = 0.0

    def set_angle(self, yaw_angle):
        goal = FollowJointTrajectoryGoal()
        goal.trajectory.joint_names = ["waist_yaw_joint"]
        yawpoint = JointTrajectoryPoint()
        self.yaw_angle = yaw_angle
        yawpoint.positions.append(self.yaw_angle)
        yawpoint.time_from_start = rospy.Duration(nsecs=1)
        goal.trajectory.points.append(yawpoint)
        self.__client.send_goal(goal)
        self.__client.wait_for_result(rospy.Duration(0.1))
        return self.__client.get_result()

if __name__ == '__main__':
    rospy.init_node("waist_test")

    wy = WaistYaw()
    wy.set_angle(math.radians(0.0))
    rospy.sleep(1.0)
    wy.set_angle(math.radians(30.0))
    rospy.sleep(1.0)
    wy.set_angle(math.radians(0.0))
    rospy.sleep(1.0)
    wy.set_angle(math.radians(-30.0))
    rospy.sleep(1.0)
    wy.set_angle(math.radians(0.0))
