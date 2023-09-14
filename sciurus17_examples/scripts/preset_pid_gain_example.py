#! /usr/bin/env python
# -*- coding: utf-8 -*-

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
import moveit_commander

from std_msgs.msg import UInt8

def preset_pid_gain(pid_gain_no):
    # サーボモータのPIDゲインをプリセットする
    # プリセットの内容はsciurus7_control/scripts/preset_reconfigure.pyに書かれている
    print("PID Gain Preset. No." + str(pid_gain_no))
    preset_no = UInt8()
    preset_no.data = pid_gain_no
    pub_preset.publish(preset_no)
    rospy.sleep(1) # PIDゲインがセットされるまで待つ


def main():
    rospy.init_node("preset_pid_gain_example")
    arm = moveit_commander.MoveGroupCommander("r_arm_group")
    arm.set_max_velocity_scaling_factor(0.1)

    # サーボモータのPIDゲインをプリセット
    preset_pid_gain(0)

    # SRDFに定義されている"r_arm_init_pose"の姿勢にする
    print("set named target : r_arm_init_pose")
    arm.set_named_target("r_arm_init_pose")
    arm.go()

    # サーボモータのPIDゲインをプリセット
    # Pゲインが小さくなるので、Sciurus17の右手を人間の手で動かすことが可能
    preset_pid_gain(4)

    # 動作確認のため数秒間待つ
    sleep_seconds = 10
    for i in range(sleep_seconds):
        print( str(sleep_seconds-i) + " counts left.")
        rospy.sleep(1)
        # 安全のため、現在の右手先姿勢を目標姿勢に変更する
        arm.set_pose_target(arm.get_current_pose())
        arm.go()

    # サーボモータのPIDゲインをプリセット
    preset_pid_gain(0)

    print("set named target : r_arm_init_pose")
    arm.set_named_target("r_arm_init_pose")
    arm.go()

    print("done")


if __name__ == '__main__':
    # PIDゲインプリセット用のPublisher
    pub_preset = rospy.Publisher("preset_gain_no", UInt8, queue_size=1)

    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass
