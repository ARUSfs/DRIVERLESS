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

# ROS msgs
from common_msgs.msg import Controls
from fssim_common.msg import State
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32,Bool,Int16

from accel_localization import AccelLocalization

sim_mode = True

class AccelControl():

    def __init__(self):
        
        self.accel_localizator = AccelLocalization()

        ### Parámetros ###
        self.params = np.array([3.1623, 16.7034, 0.0002, 2.8281],np.float64) 
        self.kp = 0.5
        self.target = 10
        self.max_cmd = 0.5
        self.min_cmd = 0
        self.track_length = 75

        ### Inicializaciones ###
        self.prev_time = 0
        self.prev_yaw = 0
        self.start_time = 0
        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.avg_speed = 0.0001
        self.i = 0
        self.braking = False
        self.AS_status = 0

        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)

        rospy.Subscriber('/perception_map', PointCloud2, self.update_route, queue_size=10)
        if sim_mode:
            rospy.Subscriber('/fssim/base_pose_ground_truth', State, self.update_speed, queue_size=1)
        else:
            rospy.Subscriber('/motor_speed', Float32, self.update_speed, queue_size=1)
        rospy.Subscriber('/can/AS_status', Int16, self.update_AS_status, queue_size=1)




    def update_speed(self, msg):
        if sim_mode:
            self.speed = math.hypot(msg.vx,msg.vy)
        else:
            self.speed = msg.data

        if sim_mode or self.AS_status == 0x02:
            if self.start_time == 0 and self.speed > 0.1:
                self.start_time = time.time()
                self.avg_speed = 0.0001
                self.i=0

            self.i += 1
            self.avg_speed = (self.avg_speed*(self.i-1) + self.speed)/self.i

            if self.start_time!=0 and (self.braking or (time.time()-self.start_time > self.track_length/self.avg_speed)):
                self.braking = True
                braking_msg = Bool()
                braking_msg.data = True
                self.braking_publisher.publish(braking_msg)
            else:
                self.acc = self.get_acc()
                self.publish_cmd()


    def update_route(self, msg: PointCloud2):
        a,b = self.accel_localizator.get_route(msg)
        self.steer = self.get_steer(a,b)


    def update_AS_status(self, msg):
        self.AS_status = msg.data


    def get_steer(self, a, b):
        dist = -b/np.sqrt(a**2 + 1)   # Distancia del coche a la trayectoria
        yaw = - math.atan(a)             # Ángulo entre la trayectoria y el coche
        beta = 0
        r = (yaw-self.prev_yaw)/(time.time()-self.prev_time)

        self.prev_yaw = yaw
        self.prev_time = time.time()

        w = np.array([dist, yaw, beta, r], np.float64) 
        steer = -np.dot(self.params, w)         # u = -K*w

        return max(min(steer, 20),-20)


    def get_acc(self):
        error = self.target - self.speed
        cmd = self.kp * error

        return max(min(cmd, self.max_cmd),0)


    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)

