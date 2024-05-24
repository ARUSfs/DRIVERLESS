#!/usr/bin/env python3

import rospy
from common_msgs.msg import Controls
from std_msgs.msg import Int16, Float32, Bool
import numpy as np
import time
import math

from mpc_handle import MPCHandle
a = 5
kp=0.5
MIN_CMD = -0.2



class Controller():

    def __init__(self):
        
        self.MPC_handler = MPCHandle()

        self.controller_mode = rospy.get_param('~controller_mode')
        topic_controller_control = rospy.get_param('~topic_controller_control')
        topic_pp_control = rospy.get_param('~topic_pp_control')
        topic_mpc_control = rospy.get_param('~topic_mpc_control')

        self.braking_ini = 0

        self.pub_cmd = rospy.Publisher(topic_controller_control, Controls, queue_size=1)
        self.pub_AS_status = rospy.Publisher('can/AS_status', Int16, queue_size=10)

        rospy.Subscriber(topic_pp_control, Controls, self.send_controllers_pp, queue_size=1)
        rospy.Subscriber(topic_mpc_control, Controls, self.send_controllers_mpc, queue_size=1)   
        rospy.Subscriber('/braking',Bool, self.start_braking, queue_size=10)  


    def send_controllers_pp(self, msg):
        if self.controller_mode=='PP' or (self.MPC_handler.FIRST_LAP and self.controller_mode=='PP-MPC'):

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logerr("PURE PURSUIT")
            self.pub_cmd.publish(msg)

    def send_controllers_mpc(self, msg):
        if (self.controller_mode=='PP-MPC' and not self.MPC_handler.FIRST_LAP):

            self.MPC_handler.par = msg.accelerator
            self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logwarn("MPC")
            self.pub_cmd.publish(msg)

    def start_braking(self, msg: Bool):
        if msg.data and self.controller_mode!='BRAKING':
            self.controller_mode = 'BRAKING'
            self.braking_time = time.time()
            self.braking_target = -1
            rospy.Subscriber('/motor_speed', Float32, self.braking_callback,queue_size=1)

    def braking_callback(self, msg):
        if self.braking_target == -1:
            self.braking_target = msg.data
            
        elif self.braking_target > 0.5 or msg.data > 0.5:
        
            self.braking_target = max(self.braking_target - a*(time.time()-self.braking_time),0)
            self.braking_time = time.time()

            rospy.logerr(self.braking_target)

            error = self.braking_target - msg.data
            cmd = kp*error
    
            control_msg = Controls()
            control_msg.accelerator = min(max(cmd,MIN_CMD),0)
            control_msg.steering = 0

            self.pub_cmd.publish(control_msg)
        
        else:
            control_msg = Controls()
            control_msg.accelerator = 0
            control_msg.steering = 0

            self.pub_cmd.publish(control_msg)

            finished_msg = Int16()
            finished_msg.data = 3
            self.pub_AS_status.publish(finished_msg)


