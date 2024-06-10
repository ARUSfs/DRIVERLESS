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

from accel_localization import AccelLocalization

KP = rospy.get_param('/accel_control/kp')
TARGET = rospy.get_param('/accel_control/target')
TRACK_LENGTH = rospy.get_param('/accel_control/track_length')
LQR_PARAMS = np.array([rospy.get_param('/accel_control/lqr_dist'),
                       rospy.get_param('/accel_control/lqr_yaw'),
                       rospy.get_param('/accel_control/lqr_beta'),
                       rospy.get_param('/accel_control/lqr_r')],np.float64) 
perception_topic = rospy.get_param('/accel_control/perception_topic')


class AccelControl():

    def __init__(self):
        
        self.accel_localizator = AccelLocalization()
        
        ### Inicializaciones ###
        self.prev_time = 0
        self.prev_yaw = 0
        self.start_time = 0
        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.x = 0
        self.y = 0
        self.yaw_car = 0
        self.avg_speed = 0.0001
        self.i = 0
        self.braking = False
        self.AS_status = 0
        self.a_media = 0
        self.b_media = 0

        ### Publicadores y suscriptores ###
        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
        self.recta_publisher = rospy.Publisher("/accel_control/recta", Point, queue_size=10)
        rospy.Subscriber(perception_topic, PointCloud2, self.update_route, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)
        rospy.Subscriber('/can/AS_status', Int16, self.update_AS_status, queue_size=1)


    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.x = msg.x
        self.y = msg.y
        self.yaw_car = msg.yaw
        
        if self.AS_status == 0x02:
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
        try:
            a,b = self.accel_localizator.get_route(msg)
            if abs(a) < 0.4 and abs(b) < 1 and self.AS_status==0x02:
                self.a_media = self.a_media*0.7 + a*0.3
                self.b_media = self.b_media*0.7 + b*0.3
            else:
                a,b = self.a_media, self.b_media
        except Exception as e:
            a,b = self.a_media, self.b_media
            rospy.logwarn(e)
        # rospy.logwarn(f"Recta: y = {a}x + {b}")
        
        # rospy.logwarn(f"{a} {b}")

        self.steer = self.get_steer(a, b)
        msg = Point()
        msg.x = a
        msg.y = b
        self.recta_publisher.publish(msg)
    
    def update_AS_status(self, msg):
        self.AS_status = msg.data


    def get_steer(self, a, b):
        dist = -(a*self.x-self.y+b)/np.sqrt(a**2 + 1)   # Distancia del coche a la trayectoria
        yaw = self.yaw_car - math.atan(a)               # Ángulo entre la trayectoria y el coche
        beta = 0
        r = (yaw-self.prev_yaw)/(time.time()-self.prev_time)

        self.prev_yaw = yaw
        self.prev_time = time.time()

        w = np.array([dist, yaw, beta, r], np.float64) 
        steer = -np.dot(LQR_PARAMS, w)         # u = -K*w

        return max(min(steer, 19.9),-19.9)


    def get_acc(self):
        error = TARGET - self.speed
        cmd = KP * error

        return max(min(cmd, 1),-1)


    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)

