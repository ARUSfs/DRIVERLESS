"""
Script to perform acceleration control based on Álvaro's localization

@Author: Ignacio Sánchez Isidro
@Date: 20220524
"""

# Python libs
import numpy as np
import rospy
import math
import time
import tf2_ros
import tf.transformations as tf

# ROS msgs
from common_msgs.msg import Controls, CarState
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32,Bool,Int16
from geometry_msgs.msg import Point

KP = rospy.get_param('/accel_control/KP')
KI = rospy.get_param('/accel_control/KI')
KD = rospy.get_param('/accel_control/KD')
TARGET = rospy.get_param('/accel_control/target')
TRACK_LENGTH = rospy.get_param('/accel_control/track_length')
REACH_TARGET_TIME = rospy.get_param('/accel_control/reach_target_time')
perception_topic = rospy.get_param('/accel_control/perception_topic')


class EBSTestControl():

    def __init__(self):
        
        ### Inicializaciones ###
        self.start_time = 0
        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.avg_speed = 0.0001
        self.i = 0
        self.braking = False
        self.integral = 0
        self.prev_t = time.time()
        self.prev_err = 0
        self.ebs_opened = False

        ### Publicadores y suscriptores ###
        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
        self.pub_AS_status = rospy.Publisher('can/AS_status', Int16, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)


    def update_state(self, msg: CarState):
        if self.ebs_opened:
            return
        
        self.update_route(msg)
        self.speed = math.hypot(msg.vx,msg.vy)
        
        if self.speed > TARGET:
            emergency_msg = Int16()
            emergency_msg.data = 4
            self.pub_AS_status.publish(emergency_msg)
            self.ebs_opened=True
            self.acc = 0
            self.publish_cmd()

        if self.start_time == 0 and self.speed > 0.1:
            self.start_time = time.time()
            self.avg_speed = 0.0001
            self.i=0

        self.i += 1
        self.avg_speed = (self.avg_speed*(self.i-1) + self.speed)/self.i

        if self.start_time!=0 and (self.braking or (time.time()-self.start_time > TRACK_LENGTH/self.avg_speed)):
            self.braking = True
            braking_msg = Bool()
            braking_msg.data = True
            self.braking_publisher.publish(braking_msg)
            self.acc=0
            self.publish_cmd()
        else:
            self.acc = self.get_acc()
            self.publish_cmd()


    def update_route(self, msg: PointCloud2):
        self.steer = 0


    def get_acc(self):
        error = self.get_target() - self.speed

        dt=time.time()-self.prev_t
        self.integral += error*dt
        derivative = (error-self.prev_err)/dt

        self.prev_t = time.time()
        self.prev_err = error

        cmd = KP*error + KI*self.integral + KD*derivative

        return max(min(cmd, 1),-1)


    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)

    def get_target(self):
        t = time.time()-self.start_time
        return min(t/REACH_TARGET_TIME,1)*TARGET