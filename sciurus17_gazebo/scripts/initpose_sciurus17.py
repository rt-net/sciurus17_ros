#!/usr/bin/env python
# coding: utf-8

import roslib
import rospy
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from std_srvs.srv import Empty

rospy.wait_for_service('/gazebo/pause_physics', timeout=120)
rospy.wait_for_service('/gazebo/set_model_configuration', timeout=120)
initialize_joints =  rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

pose_req = SetModelConfigurationRequest()
pose_req.model_name = 'sciurus17'
pose_req.urdf_param_name = '/sciurus17/robot_description'
pose_req.joint_names =     [ 'waist_yaw_joint', 'neck_yaw_joint', 'neck_pitch_joint', \
                             'r_arm_joint1', 'r_arm_joint2', 'r_arm_joint3', 'r_arm_joint4', 'r_arm_joint5', 'r_arm_joint6', 'r_arm_joint7', 'l_hand_joint', \
                             'l_arm_joint1', 'l_arm_joint2', 'l_arm_joint3', 'l_arm_joint4', 'l_arm_joint5', 'l_arm_joint6', 'l_arm_joint7', 'r_hand_joint' ]
pose_req.joint_positions = [  0.0,  0.0,  0.0, \
                                    0.0, -1.5,  0.0,  2.1,  0.0,  -2.0,  0.0,  0.0, \
                                    0.0,  1.5,  0.0, -2.1,  0.0,   2.0,  0.0,  0.0  ]
res = initialize_joints( pose_req )
