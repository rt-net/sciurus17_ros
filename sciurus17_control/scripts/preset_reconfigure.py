#!/usr/bin/env python
# coding: utf-8

import roslib
import rospy
import dynamic_reconfigure.client
from std_msgs.msg import UInt8

class PRESET_RECONFIGURE:
    def __init__( self ):
        rospy.loginfo("Wait reconfig server...")
        ### Dynamic Reconfigure 接続先一覧 ###
        self.joint_list = [
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint1"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint2"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint3"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint4"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint5"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint6"}, \
                            {"control":"/sciurus17/controller1/joints","joint":"r_arm_joint7"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint1"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint2"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint3"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint4"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint5"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint6"}, \
                            {"control":"/sciurus17/controller2/joints","joint":"l_arm_joint7"}, \
                            {"control":"/sciurus17/controller3/joints","joint":"neck_pitch_joint"},\
                            {"control":"/sciurus17/controller3/joints","joint":"neck_yaw_joint"},  \
                            {"control":"/sciurus17/controller3/joints","joint":"waist_yaw_joint"}, \
                        ]
        ### プリセット定義 - 初期値 ###
        self.preset_init = []
        for i in range(len(self.joint_list)):
            self.preset_init.append( { "p_gain": 800, "i_gain": 0, "d_gain": 0 } )
        ### プリセット定義 - 1 脱力状態 ###
        self.preset_1 = [   { "name":"r_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_pitch_joint","p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_yaw_joint",  "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"waist_yaw_joint", "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                        ]
        ### プリセット定義 - 2 左手のみ脱力 ###
        self.preset_2 = [   { "name":"r_arm_joint1",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint2",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint3",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint4",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint5",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint6",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint7",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_pitch_joint", "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_yaw_joint",   "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"waist_yaw_joint",  "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                        ]
        ### プリセット定義 - 3 右手のみ脱力 ###
        self.preset_3 = [   { "name":"r_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint1",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint2",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint3",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint4",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint5",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint6",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint7",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_pitch_joint", "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_yaw_joint",   "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"waist_yaw_joint",  "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                        ]
        ### プリセット定義 - 4 両手の脱力 ###
        self.preset_4 = [   { "name":"r_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint1",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint2",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint3",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint4",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint5",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint6",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint7",     "p_gain": 10, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_pitch_joint", "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_yaw_joint",   "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"waist_yaw_joint",  "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                        ]
        ### プリセット定義 - 5 未設定。モノを掴んだ時や、アームの形状を変えた時に合わせて設定してみてください ###
        self.preset_5 = [   { "name":"r_arm_joint1",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint2",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint3",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint4",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint5",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint6",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"r_arm_joint7",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint1",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint2",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint3",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint4",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint5",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint6",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"l_arm_joint7",     "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_pitch_joint", "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"neck_yaw_joint",   "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                            { "name":"waist_yaw_joint",  "p_gain": 800, "i_gain": 0, "d_gain": 0 },\
                        ]
        ### プリセットデータリスト ###
        self.preset_list = []
        self.preset_list.append( self.preset_init )
        self.preset_list.append( self.preset_1 )
        self.preset_list.append( self.preset_2 )
        self.preset_list.append( self.preset_3 )
        self.preset_list.append( self.preset_4 )
        self.preset_list.append( self.preset_5 )

        self.reconfigure = []
        for joint in self.joint_list:
            self.reconfigure.append( {"client":dynamic_reconfigure.client.Client( joint["control"]+"/"+joint["joint"],timeout=10 ), \
                                      "joint:":joint["joint"]} )
        rospy.loginfo("Wait sub...")
        self.subscribe = rospy.Subscriber("preset_gain_no", UInt8, self.preset_no_callback)
        rospy.loginfo("Init finished.")

    def preset_no_callback(self, no):
        joint_no = 0
        for conf in self.reconfigure:
            conf["client"].update_configuration( {"position_p_gain":self.preset_list[no.data][joint_no]["p_gain"],"position_i_gain":self.preset_list[no.data][joint_no]["i_gain"],"position_d_gain":self.preset_list[no.data][joint_no]["d_gain"]} )
            joint_no = joint_no + 1

if __name__ == '__main__':
    rospy.init_node('preset_reconfigure')
    pr = PRESET_RECONFIGURE()
    rospy.spin()
