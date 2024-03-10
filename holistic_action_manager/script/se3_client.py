from __future__ import print_function

import numpy as np

import actionlib
import rospy
import copy
import std_msgs.msg
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
        rospy.init_node('se3')
        rospy.wait_for_service("action_service_bridge")
        self.service_client = rospy.ServiceProxy('action_service_bridge', unity)
        
    def do_se3(self, arg):
        unity = geometry_msgs.msg.Pose()
        unity.position.x = -2.0
        unity.position.y = 0.5
        unity.position.z = -0.2
        unity.orientation.x = 0.0
        unity.orientation.y = 1.0
        unity.orientation.z = 0.0
        unity.orientation.w = 0.0

        holistic = True
        dur = 12

        resp = self.service_client(unity,dur,holistic)

    # def do_se3_2(self, arg):
    #     se3_goal = geometry_msgs.msg.Pose()
    #     se3_goal.position.x = 3.0
    #     se3_goal.position.y = 1.0
    #     se3_goal.position.z = 0.4
    #     se3_goal.orientation.x = 1.0
    #     se3_goal.orientation.y = 0.0
    #     se3_goal.orientation.z = 0.0
    #     se3_goal.orientation.w = 0.0

    #     rel = std_msgs.msg.Bool()
    #     rel.data = False

    #     init = std_msgs.msg.Bool()
    #     init.data = False

    #     dur = 20
    #     resp = self.service_client(se3_goal,rel,dur,init)

    # def do_se3_3(self, arg):
    #     se3_goal = geometry_msgs.msg.Pose()
    #     se3_goal.position.x = -2.0
    #     se3_goal.position.y = 2.0
    #     se3_goal.position.z = 0.5
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



    def do_init(self, arg):
        unity = geometry_msgs.msg.Pose()
        unity.position.x = -2.0
        unity.position.y = 0.5
        unity.position.z = -0.2
        unity.orientation.x = 0.0
        unity.orientation.y = 1.0
        unity.orientation.z = 0.0
        unity.orientation.w = 0.0

        rel = std_msgs.msg.Bool()
        rel.data = True
        holistic = False
        dur = 12

        resp = self.service_client(unity,dur,holistic)

    def do_quit(self, arg):
        'do quit'
        return True


    
if __name__ == '__main__':

    ControlSuiteShell().cmdloop()

