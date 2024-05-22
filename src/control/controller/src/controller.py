#!/usr/bin/env python3

import rospy
from common_msgs.msg import Controls
import numpy as np
import time
import math

from mpc_handle import MPCHandle


class Controller():

    def __init__(self):
        
        self.MPC_handler = MPCHandle()

        self.pub_cmd = rospy.Publisher('/controller/controls', Controls, queue_size=1)

        rospy.Subscriber('controls_pp', Controls, self.send_controllers_pp, queue_size=1)
        rospy.Subscriber('controls_mpc', Controls, self.send_controllers_mpc, queue_size=1)     


    def send_controllers_pp(self, msg):
        if self.MPC_handler.FIRST_LAP:

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logerr("PURE PURSUIT")
            self.pub_cmd.publish(msg)

    def send_controllers_mpc(self, msg):
        if (not self.MPC_handler.FIRST_LAP):

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logwarn("MPC")
            self.pub_cmd.publish(msg)


