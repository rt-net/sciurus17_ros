#!/usr/bin/env python
# coding: utf-8

import sys
import roslib
import rospy
import rosparam
from gazebo_msgs.srv import SetModelConfiguration
from gazebo_msgs.srv import SetModelConfigurationRequest
from sensor_msgs.msg import JointState

class MimicController(object):
    def __init__(self,model,urdf,parent,child):
        self.parent = parent
        self.child  = child
        rospy.wait_for_service('/gazebo/set_model_configuration', timeout=120)
        self.gazebo_service =  rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)

        self.mimic_req = SetModelConfigurationRequest()
        self.mimic_req.model_name      = model
        self.mimic_req.urdf_param_name = urdf
        self.mimic_req.joint_names     = [ self.child ]
        self.mimic_req.joint_positions = [  0.0  ]
        rospy.Subscriber("joint_states", JointState, self.joint_states_callback, queue_size=1)

    def joint_states_callback(self, msg):
        for i in range(len(msg.name)):
            if self.parent == msg.name[i]:
                mimic_position = msg.position[i]
                self.mimic_req.joint_positions[0] = mimic_position
                res = self.gazebo_service( self.mimic_req )

if __name__ == '__main__':
    if len(sys.argv) < 5:
        print("usage: mimic_controller.py model urdf parent child")
    else:
        rospy.init_node('mimic_controller')
        model  = sys.argv[1]
        urdf   = sys.argv[2]
        parent = sys.argv[3]
        child  = sys.argv[4]
        mimic = MimicController( model, urdf, parent, child )
        rospy.spin()
