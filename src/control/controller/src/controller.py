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

        self.controller_mode = rospy.get_param('~controller_mode')
        topic_controller_control = rospy.get_param('~topic_controller_control')
        topic_pp_control = rospy.get_param('~topic_pp_control')
        topic_mpc_control = rospy.get_param('~topic_mpc_control')

        self.pub_cmd = rospy.Publisher(topic_controller_control, Controls, queue_size=1)

        rospy.Subscriber(topic_pp_control, Controls, self.send_controllers_pp, queue_size=1)
        rospy.Subscriber(topic_mpc_control, Controls, self.send_controllers_mpc, queue_size=1)   

        if self.controller_mode == 'MPC':
            time.sleep(2)
            self.MPC_handler.set_virtual_mpc_route()  


    def send_controllers_pp(self, msg):
        if self.controller_mode=='PP' or (self.MPC_handler.FIRST_LAP and self.controller_mode=='PP-MPC'):

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logerr("PURE PURSUIT")
            self.pub_cmd.publish(msg)

    def send_controllers_mpc(self, msg):
        if ((self.controller_mode=='PP-MPC' or self.controller_mode=='MPC') and not self.MPC_handler.FIRST_LAP):

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logwarn("MPC")
            self.pub_cmd.publish(msg)


