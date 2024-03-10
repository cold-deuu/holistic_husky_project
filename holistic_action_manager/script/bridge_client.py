from __future__ import print_function

import numpy as np


import actionlib
import rospy
import copy
import std_msgs.msg
import holistic_action_manager.msg
from sensor_msgs.msg import JointState
import tf
from tf.transformations import quaternion_matrix
import math
import geometry_msgs.msg
from std_msgs.msg import Float64
from copy import deepcopy 
import cmd, sys, os
from holistic_action_manager.srv import unity


class Bridge:
    def __init__(self):
        rospy.init_node('bridge')
        self.service_server = rospy.Service('action_service_bridge', unity, self.goalCallback)
        self.wait = False
        self.holistic = False
        self.holistic_ctrl_client = actionlib.SimpleActionClient('/holistic_action_manager/holistic_control', holistic_action_manager.msg.HolisticAction)
        self.holistic_ctrl_client.wait_for_server()

        self.joint_posture_client = actionlib.SimpleActionClient('/holistic_action_manager/joint_posture', holistic_action_manager.msg.JointPostureAction)
        self.joint_posture_client.wait_for_server()
        

    def goalCallback(self, req):
        self.goal = holistic_action_manager.msg.HolisticGoal
        self.goal.target_pose = geometry_msgs.msg.Pose()

        self.goal.target_pose.position.x = req.pose.position.x
        self.goal.target_pose.position.y = req.pose.position.y
        self.goal.target_pose.position.z = req.pose.position.z
        self.goal.target_pose.orientation.x = req.pose.orientation.x
        self.goal.target_pose.orientation.y = req.pose.orientation.y
        self.goal.target_pose.orientation.z = req.pose.orientation.z
        self.goal.target_pose.orientation.w = req.pose.orientation.w

        self.goal.duration = req.duration
        self.holistic = req.holistic
        self.wait = True
        return True

    def compute(self):
        if self.holistic:
            self.holistic_ctrl_client.send_goal(self.goal)
            self.holistic_ctrl_client.wait_for_result()
            if (self.holistic_ctrl_client.get_result()):
                print ("action succeed")
            else:
                print ("action failed")

        else:
            arm_joint_goal = holistic_action_manager.msg.JointPostureGoal
            arm_joint_goal.target_joints = JointState()
            arm_joint_goal.target_joints.position = np.array([0,0,0,-np.pi/2,0,np.pi/2,0])
            arm_joint_goal.duration = 3.0
            self.joint_posture_client.send_goal(arm_joint_goal)
            self.joint_posture_client.wait_for_result()
            if (self.joint_posture_client.get_result()):
                print ("action succeed")
            else:
                print ("action failed")

        self.wait = False



    
if __name__ == '__main__':
    bridge = Bridge()

    while True:
        if bridge.wait:
            bridge.compute()
            print("A")
    if bridge.wait:
        bridge.compute()
    else:
        rospy.spin()