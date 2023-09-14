#!/usr/bin/env python

# Copyright 2018 RT Corporation
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

import sys
import argparse

import rospy
import time
import actionlib
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)
import math

CONTROL_R = 0
CONTROL_L = 1

class GripperClient(object):
    def __init__(self):
        self._clientR = actionlib.SimpleActionClient("/sciurus17/controller1/right_hand_controller/gripper_cmd",GripperCommandAction)
        self._clientL = actionlib.SimpleActionClient("/sciurus17/controller2/left_hand_controller/gripper_cmd",GripperCommandAction)

        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()

        # Wait 10 Seconds for the gripper action server to start or exit
        self._clientR.wait_for_server(rospy.Duration(5.0))
        if not self._clientR.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper R Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()

        # Wait 10 Seconds for the gripper action server to start or exit
        self._clientL.wait_for_server(rospy.Duration(5.0))
        if not self._clientL.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Exiting - Gripper L Action Server Not Found")
            rospy.signal_shutdown("Action Server not found")
            sys.exit(1)
        self.clear()


    def command(self, position, effort, type):
        if type == CONTROL_R:
            self._goalR.command.position = position
            self._goalR.command.max_effort = effort
            self._clientR.send_goal(self._goalR,feedback_cb=self.feedbackR)
        if type == CONTROL_L:
            self._goalL.command.position = position
            self._goalL.command.max_effort = effort
            self._clientL.send_goal(self._goalL,feedback_cb=self.feedbackL)

    def feedbackR(self,msg):
        print("feedbackR")
        print(msg)

    def feedbackL(self,msg):
        print("feedbackL")
        print(msg)

    def stop(self):
        self._clientR.cancel_goal()
        self._clientL.cancel_goal()

    def wait(self, type, timeout=0.1 ):
        if type == CONTROL_R:
            self._clientR.wait_for_result(timeout=rospy.Duration(timeout))
            return self._clientR.get_result()
        if type == CONTROL_L:
            self._clientL.wait_for_result(timeout=rospy.Duration(timeout))
            return self._clientL.get_result()

    def clear(self):
        self._goalR = GripperCommandGoal()
        self._goalL = GripperCommandGoal()

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)

    print("Initializing node... ")
    rospy.init_node("gipper_action_client")

    gc = GripperClient()

    # Open grippers
    print("Test - Open Grippers")
    gripper = 26.0
    gc.command(math.radians(gripper),0.1,CONTROL_R)
    gripper = -26.0
    gc.command(math.radians(gripper),0.1,CONTROL_L)
    result = gc.wait(CONTROL_R,1.0)
    print(result)
    result = gc.wait(CONTROL_L,1.0)
    print(result)
    time.sleep(1)

    # Close grippers
    print("Test - Close Grippers")
    gripper = 0.0
    gc.command(math.radians(gripper),0.5,CONTROL_R)
    gripper = 0.0
    gc.command(math.radians(gripper),0.5,CONTROL_L)
    result = gc.wait(CONTROL_R,1.0)
    print(result)
    result = gc.wait(CONTROL_L,1.0)
    print(result)
    time.sleep(1)

    print("Exiting - Gripper Action Test Example Complete")

if __name__ == "__main__":
    main()
