#!/usr/bin/env python3

import rospy
from common_msgs.msg import Controls, CarState
from std_msgs.msg import Int16, Float32, Bool
import numpy as np
import time
import math

from mpc_handle import MPCHandle

# for braking
BRAKING_KP = rospy.get_param('/controller/braking_kp')
DECELERATION = rospy.get_param('/controller/deceleration')
MIN_BRAKING_CMD = rospy.get_param('/controller/min_braking_cmd') 

# for accelerating
MAX_CMD = rospy.get_param('/controller/max_cmd')
MIN_CMD = rospy.get_param('/controller/min_cmd')
MIN_VEL = rospy.get_param('/controller/min_vel')
AMPLITUDE = rospy.get_param('/controller/amplitude')



class Controller():

    def __init__(self):
        
        self.MPC_handler = MPCHandle()

        self.controller_mode = rospy.get_param('/controller/controller_mode')
        topic_controller_control = rospy.get_param('/controller/topic_controller_control')
        topic_pp_control = rospy.get_param('/controller/topic_pp_control')
        topic_mpc_control = rospy.get_param('/controller/topic_mpc_control')


        self.braking_target = -1
        self.braking_ini = 0
        self.AS_status = 0
        self.v = 0

        self.pub_cmd = rospy.Publisher(topic_controller_control, Controls, queue_size=1)
        self.pub_AS_status = rospy.Publisher('can/AS_status', Int16, queue_size=10)

        rospy.Subscriber('/can/AS_status', Int16, self.update_AS_status, queue_size=1)
        rospy.Subscriber('/car_state/state', CarState, self.state_callback,queue_size=1)
        rospy.Subscriber(topic_pp_control, Controls, self.send_controllers_pp, queue_size=1)
        rospy.Subscriber(topic_mpc_control, Controls, self.send_controllers_mpc, queue_size=1)   
        rospy.Subscriber('/braking',Bool, self.start_braking, queue_size=10)  

    def update_AS_status(self, msg: Int16):
        if msg.data == 2:
            self.AS_status = msg.data

    def send_controllers_pp(self, msg):
        self.steer = msg.steering
        if self.AS_status == 0x02 and (self.controller_mode=='PP' or (self.MPC_handler.FIRST_LAP and self.controller_mode=='PP-MPC')):
            #limit command
            msg.accelerator = min(max(msg.accelerator,MIN_CMD),MAX_CMD)
            self.MPC_handler.par = msg.accelerator

            if self.v < MIN_VEL:
                msg.steering = 0
                self.MPC_handler.delta = 0
            else:
                msg.steering = min(max(msg.steering,-AMPLITUDE),AMPLITUDE)
                self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logerr("PURE PURSUIT")
            self.pub_cmd.publish(msg)

    def send_controllers_mpc(self, msg):
        self.steer = msg.steering
        if self.AS_status == 0x02 and (self.controller_mode=='PP-MPC' and not self.MPC_handler.FIRST_LAP):
            # limit command
            msg.accelerator = min(max(msg.accelerator,MIN_CMD),MAX_CMD)
            self.MPC_handler.par = msg.accelerator

            if self.v < MIN_VEL:
                msg.steering = 0
                self.MPC_handler.delta = 0
            else:
                msg.steering = min(max(msg.steering,-AMPLITUDE),AMPLITUDE)
                self.MPC_handler.delta = math.radians(msg.steering)

            # rospy.logwarn("MPC")
            self.pub_cmd.publish(msg)

    def start_braking(self, msg: Bool):
        if msg.data and self.controller_mode!='BRAKING':
            self.controller_mode = 'BRAKING'
            self.braking_time = time.time()

    def state_callback(self, msg:CarState):
        self.v = math.hypot(msg.vx,msg.vy)

        if self.controller_mode == 'BRAKING':
            if self.braking_target == -1:
                self.braking_target = self.v
                
            elif self.braking_target > 0.5 or self.v > 0.5:
            
                self.braking_target = max(self.braking_target - DECELERATION*(time.time()-self.braking_time),0)
                self.braking_time = time.time()

                error = self.braking_target - self.v
                cmd = BRAKING_KP*error
        
                control_msg = Controls()
                control_msg.accelerator = min(max(cmd,MIN_BRAKING_CMD),0)
                control_msg.steering = self.steer

                self.pub_cmd.publish(control_msg)
            
            else:
                control_msg = Controls()
                control_msg.accelerator = 0
                control_msg.steering = self.steer

                self.pub_cmd.publish(control_msg)

                self.AS_status = 3

                finished_msg = Int16()
                finished_msg.data = 3
                self.pub_AS_status.publish(finished_msg)


