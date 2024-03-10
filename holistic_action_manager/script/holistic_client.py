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


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'



class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rospy.init_node('control')
        self.holistic_ctrl_client = actionlib.SimpleActionClient('/holistic_action_manager/holistic_control', holistic_action_manager.msg.HolisticAction)
        self.holistic_ctrl_client.wait_for_server()

        self.joint_posture_client = actionlib.SimpleActionClient('/holistic_action_manager/joint_posture', holistic_action_manager.msg.JointPostureAction)
        self.joint_posture_client.wait_for_server()
    

    def do_se3(self, arg):
        se3_goal = holistic_action_manager.msg.HolisticGoal

        se3_goal.duration = 15.0

        se3_goal.target_pose = geometry_msgs.msg.Pose()
        se3_goal.target_pose.position.x = -4.0
        se3_goal.target_pose.position.y = 0.0
        se3_goal.target_pose.position.z = 0.4
        se3_goal.target_pose.orientation.x = 0.0
        se3_goal.target_pose.orientation.y = 1.0
        se3_goal.target_pose.orientation.z = 0.0
        se3_goal.target_pose.orientation.w = 0.0

        self.holistic_ctrl_client.send_goal(se3_goal)
        self.holistic_ctrl_client.wait_for_result()
        if (self.holistic_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_home(self,arg):
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

    def do_pose(self,arg):
        arm_joint_goal = holistic_action_manager.msg.JointPostureGoal
        arm_joint_goal.target_joints = JointState()
        arm_joint_goal.target_joints.position = np.array([0,np.pi/3,0,-np.pi/2,0,np.pi/2,0])
        arm_joint_goal.duration = 3.0

        self.joint_posture_client.send_goal(arm_joint_goal)
        self.joint_posture_client.wait_for_result()
        if (self.joint_posture_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")

    def do_reach(self, arg):
        se3_goal = holistic_action_manager.msg.HolisticGoal

        se3_goal.duration = 5.0

        se3_goal.target_pose = geometry_msgs.msg.Pose()
        se3_goal.target_pose.position.x = -3.0
        se3_goal.target_pose.position.y = 0.0
        se3_goal.target_pose.position.z = 0.4
        se3_goal.target_pose.orientation.x = 0.0
        se3_goal.target_pose.orientation.y = 1.0
        se3_goal.target_pose.orientation.z = 0.0
        se3_goal.target_pose.orientation.w = 0.0

        self.holistic_ctrl_client.send_goal(se3_goal)
        self.holistic_ctrl_client.wait_for_result()
        if (self.holistic_ctrl_client.get_result()):
            print ("action succeed")
        else:
            print ("action failed")
            
    # def do_se3_2(self, arg):
    #     se3_goal = geometry_msgs.msg.Pose()
    #     se3_goal.position.x = 3.0
    #     se3_goal.position.y = 2.0
    #     se3_goal.position.z = 0.7
    #     se3_goal.orientation.x = 0.0
    #     se3_goal.orientation.y = 1.0
    #     se3_goal.orientation.z = 0.0
    #     se3_goal.orientation.w = 0.0

    #     rel = std_msgs.msg.Bool()
    #     rel.data = False

    #     init = std_msgs.msg.Bool()
    #     init.data = False

    #     dur = 15
    #     resp = self.service_client(se3_goal,rel,dur,init)

    # def do_se3_3(self, arg):
    #     se3_goal = geometry_msgs.msg.Pose()
    #     se3_goal.position.x = -2.0
    #     se3_goal.position.y = 2.0
    #     se3_goal.position.z = 0.5
    #     se3_goal.orientation.x = 0.0
    #     se3_goal.orientation.y = 0.0
    #     se3_goal.orientation.z = 1.0
    #     se3_goal.orientation.w = 0.0

    #     rel = std_msgs.msg.Bool()
    #     rel.data = False

    #     init = std_msgs.msg.Bool()
    #     init.data = False
    #     dur = 15

    #     resp = self.service_client(se3_goal,rel,dur,init)



    # def do_init(self, arg):
    #     se3_goal = geometry_msgs.msg.Pose()
    #     se3_goal.position.x = 0
    #     se3_goal.position.y = 0
    #     se3_goal.position.z = 0
    #     se3_goal.orientation.x = 0.0
    #     se3_goal.orientation.y = 1.0
    #     se3_goal.orientation.z = 0.0
    #     se3_goal.orientation.w = 0.0

    #     rel = std_msgs.msg.Bool()
    #     rel.data = False

    #     init = std_msgs.msg.Bool()
    #     init.data = True

    #     dur = 3
    #     resp = self.service_client(se3_goal,rel,dur,init)

    def do_quit(self, arg):
        'do quit'
        return True


    
if __name__ == '__main__':

    ControlSuiteShell().cmdloop()

