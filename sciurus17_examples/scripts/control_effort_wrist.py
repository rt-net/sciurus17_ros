#! /usr/bin/env python
# coding: utf-8

# Copyright 2020 RT Corporation
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

import math
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


class PIDController(object):
    def __init__(self, p_gain, i_gain, d_gain):

        self._p_gain = p_gain
        self._i_gain = i_gain
        self._d_gain = d_gain

        self._error_1 = 0.0
        self._error_2 = 0.0
        self._output = 0.0

    def update(self, current, target):
        error = target - current

        delta_output = self._p_gain * (error - self._error_1)
        delta_output += self._i_gain * (error)
        delta_output += self._d_gain * (error - 2*self._error_1 + self._error_2)

        self._output += delta_output

        self._error_2 = self._error_1
        self._error_1 = error

        return self._output


def stop():
    pub_wrist_current.publish(0.0)
    print("TORQUE_OFF")

joint_state = JointState()
def joint_state_callback(msg):
    global joint_state
    joint_state = msg

def main():
    global joint_state

    # Set P and D gain to avoid the wrist oscillation.
    pid_controller = PIDController(0.5, 0.0, 3.0)

    r = rospy.Rate(60)
    start_time = rospy.Time.now().secs
    target_angle = math.radians(90)
    while not rospy.is_shutdown():
        if len(joint_state.position) < 7:  # Wrist is the 7th joint.
            continue

        wrist_angle = joint_state.position[6]

        # Switch the target angle every 5 seconds.
        present_time = rospy.Time.now().secs
        if present_time - start_time > 5:
            start_time = present_time
            target_angle *= -1.0

        target_effort = pid_controller.update(wrist_angle, target_angle)
        pub_wrist_current.publish(target_effort)
        r.sleep()


if __name__ == '__main__':
    rospy.init_node("control_effort_wrist")

    sub_joint_state = rospy.Subscriber(
        "/sciurus17/controller2/joint_states",
        JointState,
        joint_state_callback,
        queue_size=1)

    pub_wrist_current = rospy.Publisher(
        "/sciurus17/controller2/left_wrist_controller/command",
        Float64,
        queue_size=1)

    rospy.on_shutdown(stop)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass

