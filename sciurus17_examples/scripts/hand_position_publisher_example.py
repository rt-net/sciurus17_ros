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

import roslib
import rospy
import math
import tf
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('hand_position_pub')

    listener = tf.TransformListener()

    left_pose_pub = rospy.Publisher('/sciurus17/hand_pos/left', geometry_msgs.msg.Pose,queue_size=1)
    right_pose_pub = rospy.Publisher('/sciurus17/hand_pos/right', geometry_msgs.msg.Pose,queue_size=1)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('base_link', 'l_link7', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #rospy.loginfo("pos[x=%f y=%f z=%f] rot[x=%f y=%f z=%f w=%f]" % (trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        left_pose = geometry_msgs.msg.Pose()
        left_pose.position.x = trans[0] 
        left_pose.position.y = trans[1] 
        left_pose.position.z = trans[2] 
        left_pose.orientation.x = rot[0] 
        left_pose.orientation.y = rot[1] 
        left_pose.orientation.z = rot[2] 
        left_pose.orientation.w = rot[3] 
        left_pose_pub.publish( left_pose )

        try:
            (trans,rot) = listener.lookupTransform('base_link', 'r_link7', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #rospy.loginfo("pos[x=%f y=%f z=%f] rot[x=%f y=%f z=%f w=%f]" % (trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]))
        right_pose = geometry_msgs.msg.Pose() 
        right_pose.position.x = trans[0] 
        right_pose.position.y = trans[1] 
        right_pose.position.z = trans[2] 
        right_pose.orientation.x = rot[0] 
        right_pose.orientation.y = rot[1] 
        right_pose.orientation.z = rot[2] 
        right_pose.orientation.w = rot[3] 
        right_pose_pub.publish( right_pose )

        rate.sleep()

