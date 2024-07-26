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
from std_msgs.msg import Float32,Bool,Int16, Float32MultiArray
from geometry_msgs.msg import Point

from accel_localization import AccelLocalization

KP = rospy.get_param('/accel_control/KP')
KI = rospy.get_param('/accel_control/KI')
KD = rospy.get_param('/accel_control/KD')
TARGET = rospy.get_param('/accel_control/target')
PENDIENTE = rospy.get_param('/accel_control/pendiente')
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
        self.start_time = 0
        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.yaw_car = 0
        self.avg_speed = 0.0001
        self.i = 0
        self.braking = False
        self.a_media = 0
        self.b_media = 0
        self.prev_t = time.time()
        self.prev_err = 0
        self.integral = 0
        self.r = 0

        ### Publicadores y suscriptores ###
        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
        self.recta_publisher = rospy.Publisher("/accel_control/recta", Point, queue_size=10)
        rospy.Subscriber(perception_topic, PointCloud2, self.update_route, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)
        self.phi_dist_pub = rospy.Publisher('/phi_dist', Float32MultiArray, queue_size=1)

    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.yaw_car = msg.yaw
        self.r = msg.r

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
            if abs(a) < 0.5 and abs(b) < 2:
                self.a_media = self.a_media*0.7 + a*0.3
                self.b_media = self.b_media*0.7 + b*0.3
            else:
                a,b = self.a_media, self.b_media
        except Exception as e:
            a,b = self.a_media, self.b_media
            rospy.logwarn(e)

        self.steer = self.get_steer(a, b)
        msg = Point()
        msg.x = a
        msg.y = b
        self.recta_publisher.publish(msg)


    def get_steer(self, a, b):
        dist = -b/np.sqrt(a**2 + 1)   # Distancia del coche a la trayectoria
        yaw = - math.atan(a)               # Ángulo entre la trayectoria y el coche
        beta = 0

        phi_dist_msg = Float32MultiArray()
        phi_dist_msg.data.append(yaw)
        phi_dist_msg.data.append(dist)
        self.phi_dist_pub.publish(phi_dist_msg)

        w = np.array([dist, yaw, beta, self.r], np.float64) 
        steer = -np.dot(LQR_PARAMS, w)         # u = -K*w

        return max(min(steer, 20),-20)


    def get_acc(self):
        error = self.get_target() - self.speed

        dt=time.time()-self.prev_t
        if self.speed>0.1:
            self.integral += error*dt
        derivative = (error-self.prev_err)/dt

        self.prev_t = time.time()
        self.prev_err = error

        cmd = KP*error + KI*self.integral + KD*derivative

        return max(min(cmd/230, 1),-1)


    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)

   
    def get_target(self):
        t = time.time() - self.start_time
        return min(t*PENDIENTE+TARGET/10, TARGET)
