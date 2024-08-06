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
from common_msgs.msg import Controls, CarState, Trajectory
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32,Bool,Int16, Float32MultiArray
from geometry_msgs.msg import Point

STANLEY_COEF = rospy.get_param('/stanley_control/stanley_coef')
K_DELTA = rospy.get_param('/stanley_control/k_delta')
K_YAW_RATE = rospy.get_param('/stanley_control/k_yaw_rate')

KP = rospy.get_param('/stanley_control/KP')
KI = rospy.get_param('/stanley_control/KI')
KD = rospy.get_param('/stanley_control/KD')



class StanleyControl():

    def __init__(self):

        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.yaw = 0
        self.r = 0

        self.i = 0
        self.integral = 0
        self.prev_err = 0
        self.steer_diff = 0
        self.prev_steer = 0
        self.prev_t = time.time()
        self.braking = False
        
        self.s = None
        self.k = None
        self.speed_profile = None
        self.has_route = False

        self.cmd_publisher = rospy.Publisher('/controls_stanley', Controls, queue_size=1) 
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=1)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=10)
        rospy.Subscriber('/i_phi_dist', Float32MultiArray, self.callback, queue_size=1)
        rospy.Subscriber('/controller/sk', Trajectory, self.update_route, queue_size=10)
        rospy.Subscriber('/steering/epos_info', Float32MultiArray, self.update_steer, queue_size=1)
        rospy.Subscriber('/braking',Bool, self.braking_callback, queue_size=10)

        
    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.yaw = msg.yaw
        self.r = msg.r

    def braking_callback(self, msg: Bool):
        self.braking = msg.data

    def update_route(self, msg: Trajectory):
        if not self.has_route:
            self.s = np.array([p.x for p in msg.trajectory])
            self.k = np.array([p.y for p in msg.trajectory])

            speed_profile = [0 for _ in range(len(self.s))]
            ax_max = 3
            ay_max = 4
            v_max = 8
            v_grip = [min(np.sqrt(ay_max/np.abs(k+0.0001)),v_max) for k in self.k]
            speed_profile[0] = v_max
            for j in range(1,len(speed_profile)):
                ds = self.s[j]-self.s[j-1]
                speed_profile[j] = np.sqrt(speed_profile[j-1]**2 + 2*ax_max*ds)
                if speed_profile[j] > v_grip[j]:
                    speed_profile[j] = v_grip[j]
            for j in range(len(speed_profile)-2,-1,-1):
                v_max_braking = np.sqrt(speed_profile[j+1]**2 + 2*ax_max*ds)
                if speed_profile[j] > v_max_braking:
                    speed_profile[j] = v_max_braking

            rospy.loginfo(speed_profile)
            self.speed_profile = speed_profile

            self.has_route = True

    def callback(self, msg: Float32MultiArray):
        if self.has_route:
            self.i = int(msg.data[0])
            self.phi = msg.data[1]
            self.dist = msg.data[2]

            self.acc = self.get_acc()
            self.steer = self.get_steer()
            self.publish_cmd()

    
    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)

        if(not self.braking):
            target_speed = Float32()
            target_speed.data = self.speed_profile[self.i]
            self.target_speed_pub.publish(target_speed)


    def get_acc(self):
        
        error = self.speed_profile[self.i] - self.speed

        dt=time.time()-self.prev_t
        if self.speed > 0.1:
            self.integral += error*dt
        derivative = (error-self.prev_err)/dt

        self.prev_t = time.time()
        self.prev_err = error
        
        cmd = KP*error + KI*self.integral + KD*derivative

        rospy.loginfo(self.speed)
        rospy.loginfo([error,self.integral,derivative])

        return max(min(cmd/230, 1),-1)
    
    def update_steer(self, msg: Float32MultiArray):
        self.steer_diff = self.prev_steer - msg.data[1]
        self.prev_steer = msg.data[1]

    
    def get_steer(self):

        r_target = self.speed*self.k[self.i]

        phi_ss = 250*self.speed*r_target/(-20000*(1+0.55/0.45))

        ### STANLEY CONTROL ###
        delta = -self.phi -phi_ss - np.arctan(STANLEY_COEF*self.dist/self.speed_profile[self.i])
        delta += K_DELTA*self.steer_diff + K_YAW_RATE*(r_target - self.r) 
        delta = math.degrees(delta)
        
     
        return max(-20,min(20,delta))
    



    
