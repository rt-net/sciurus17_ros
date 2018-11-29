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

if __name__ == '__main__':
    rospy.init_node("neck_test")

    np = NeckPitch()
    try:
        pitch = float(sys.argv[1])
    except:
        pass
    np.set_angle(math.radians(0.0), math.radians(-60.0))
    rospy.sleep(1.0)
    np.set_angle(math.radians(0.0), math.radians(60.0))
    rospy.sleep(1.0)
    np.set_angle(math.radians(0.0), math.radians(0.0))
    rospy.sleep(1.0)
    np.set_angle(math.radians(-90.0), math.radians(0.0))
    rospy.sleep(1.0)
    np.set_angle(math.radians(90.0), math.radians(0.0))
    rospy.sleep(1.0)
    np.set_angle(math.radians(0.0), math.radians(0.0))
