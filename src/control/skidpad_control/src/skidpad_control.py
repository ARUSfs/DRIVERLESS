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

# ROS msgs
from common_msgs.msg import Controls, CarState
from fssim_common.msg import State
from visualization_msgs.msg import Marker
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32,Bool,Int16, Float32MultiArray
from geometry_msgs.msg import Point
from sklearn.cluster import KMeans, DBSCAN
from geometry_msgs.msg import PointStamped

from skidpad_localization import SkidpadLocalization

KP = rospy.get_param('/skidpad_control/kp')
KI = rospy.get_param('/skidpad_control/ki')
KD = rospy.get_param('/skidpad_control/kd')
TARGET = rospy.get_param('/skidpad_control/target')

k_mu = rospy.get_param('/skidpad_control/k_mu')
k_phi = rospy.get_param('/skidpad_control/k_phi')
k_r = rospy.get_param('/skidpad_control/k_r')
delta_correction = rospy.get_param('/skidpad_control/delta_correction')


class SkidpadControl():

    def __init__(self):
        
        self.skidpad_localizator = SkidpadLocalization()

        ### Inicializaciones ###

        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.braking = False
        self.integral = 0
        self.prev_err = 0
        self.prev_t = time.time()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.centers = []
        self.start_time = None
        self.calib_time = 2
        self.calibrated = False

        self.i = 0

 
        self.N = 100
        r=9.125
        d = 2*math.pi*r/self.N

        self.plantilla = np.array([[-20 + d*i,0] for i in range(int(20/d)+1)]+2*[[r * np.sin(2 * np.pi * i / self.N), -9.125+r * np.cos(2 * np.pi * i / self.N)] for i in range(self.N)]+2*[[r * np.sin(2 * np.pi * i / self.N), 9.125-r * np.cos(2 * np.pi * i / self.N)] for i in range(self.N)]+[[d*i,0] for i in range(int(15/d)+1)])
        self.route = None
        
    

        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
        self.route_pub = rospy.Publisher('/skidpad_route',Marker,queue_size=1)
        self.pubb = rospy.Publisher('/phi_dist', Float32MultiArray,queue_size=1)
        rospy.Subscriber('/perception_map', PointCloud2, self.update_route, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)
        rospy.Subscriber('/steering/epos_info', Float32MultiArray, self.update_steer, queue_size=1)



    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.pos = np.array([msg.x,msg.y])

        if self.calibrated:
            self.acc = self.get_acc()
            self.steer = self.get_steer(msg)
            self.publish_cmd()
    
    def update_steer(self, msg: Float32MultiArray):
        self.delta_real = msg.data[1]


    def update_route(self, msg: PointCloud2):
        t = time.time()
        if self.start_time == None:
            self.start_time = time.time()
        
        if time.time()-self.start_time < self.calib_time:
            C1,C2 = self.skidpad_localizator.get_centers(msg)
            rospy.logwarn([C1,C2])
            self.centers.append(C1)
            self.centers.append(C2)
        elif not self.calibrated:

            epsilon = 1  # Distancia máxima entre puntos para considerarlos vecinos
            min_samples = int(5*self.calib_time)  # Número mínimo de puntos para formar un clúster

            dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
            dbscan.fit(np.array(self.centers))

            centroid1_mask = dbscan.labels_ == 0
            centroid2_mask = dbscan.labels_ == 1

            centroid1 = np.mean(np.array(self.centers)[centroid1_mask],axis=0)
            centroid2 = np.mean(np.array(self.centers)[centroid2_mask],axis=0)

            if np.abs(18.25 - np.linalg.norm(centroid1-centroid2)) < 1:
                self.calibrated = True

                center = [(centroid1[0]+centroid2[0])/2,(centroid1[1]+centroid2[1])/2]
                yaw = np.pi/2-np.arctan2(centroid1[1]-centroid2[1],centroid1[0]-centroid2[0])
                
                # con esto el giro está entre -90º y 90º
                if yaw < -np.pi/2:
                    yaw = yaw+np.pi
                elif yaw > np.pi/2:
                    yaw = yaw-np.pi

                rot = np.array([[math.cos(yaw),-math.sin(yaw)],[math.sin(yaw),math.cos(yaw)]])
                self.route = self.plantilla @ rot + np.array(center)

                marker = Marker()
                marker.header.frame_id = "fssim/vehicle/cog"
                marker.id = 102
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.2
                marker.color.r = 1
                marker.color.a = 1.0
                marker.points = [Point(p[0], p[1], 0.0) for p in self.route]
                self.route_pub.publish(marker)
            else:
                self.centers = []
                self.start_time = time.time()

        # rospy.loginfo(time.time()-t)



    def get_acc(self):
        error = TARGET - self.speed

        dt=time.time()-self.prev_t
        if self.speed > 0.1:
            self.integral += error*dt
        derivative = (error-self.prev_err)/dt

        self.prev_t = time.time()
        self.prev_err = error

        cmd = KP*error + KI*self.integral + KD*derivative

        return max(min(cmd/230, 1),-1)


    def publish_cmd(self):
        controls = Controls()
        controls.steering = self.steer
        if self.braking:
            controls.accelerator = 0
        controls.accelerator = self.acc
        self.cmd_publisher.publish(controls)


    def get_steer(self,msg: CarState):
        if self.i+self.N/2 < len(self.route):
            dist = np.linalg.norm(self.route[self.i:self.i+int(self.N/2),:]-self.pos,axis=1)
            self.i+=np.argmin(dist)
        elif self.i< len(self.route)-1:
            dist = np.linalg.norm(self.route[self.i:,:]-self.pos,axis=1)
            self.i+=np.argmin(dist)

        i = self.i

        x = self.route[:,0]
        y = self.route[:,1]

        if x[i]!=x[i+1]:
            recta=(y[self.i+1]-y[i])/(x[i+1]-x[i])*(self.pos[0]-x[i])+y[i]-msg.y
            signo_d=np.sign(recta)*np.sign(x[i]-x[i+1])
        else:
            signo_d=np.sign(y[i+1]-y[i])*np.sign(self.pos[0]-x[i])

        d_min=np.min(dist)*signo_d

        #corrected_yaw = (msg.yaw+np.pi)%(2*np.pi) - np.pi
        theta = np.arctan2(y[(i+5)%len(x)]-y[i],x[(i+5)%len(x)]-x[i])
        #phi = corrected_yaw - theta
        #phi_corrected = corrected_yaw - theta if np.abs(phi)<2 else (phi-2*np.pi if phi>0 else phi+2*np.pi)
        phi_corrected = ((msg.yaw-theta)+np.pi)%(2*np.pi) - np.pi 
       
        self.dist = d_min+0.0001
        self.phi = phi_corrected
        self.vx = msg.vx
        self.vy = msg.vy
        self.r = msg.r

        d = 2*math.pi*9.125/self.N


        if self.i > 4*self.N+int(25/d):
            self.braking=True
            braking_msg = Bool()
            braking_msg.data = True
            self.braking_publisher.publish(braking_msg)
            return self.steer

        self.si = self.i*d
        self.k = self.kk()

        r_target = self.vx*self.k

        ### BASE CONTROL + CORRECTION ###
        # delta = delta_correction*math.degrees(np.arctan(self.k*1.535)) - k_mu*((self.dist**3+0.1*self.dist)) - k_phi*(self.phi+2*np.arctan(self.k*1.535/2)) + k_r*(r_target - self.r)
        
        ### STANLEY CONTROL ###
        coef = 0.2
        delta = -self.phi - np.arctan(coef*self.dist/TARGET)
        # OPTIONAL CORRECTION
        # delta += -0.02*(delta-self.delta_real) + 0.2*(r_target - self.r)    
        delta = math.degrees(delta)
        delta = max(-20,min(20,delta))
        
        params = Float32MultiArray()
        params.data.append(self.phi)
        params.data.append(self.dist)
        self.pubb.publish(params)
     
        return delta
    
    def kk(self):
        s11=5
        s12=2
        s21=6
        s22=4
        f=2
        if self.si < 19-s11:
            self.k=0
        elif self.si < 19+s12:
            self.k = -(1/9.125)*((self.si-19+s11)/(s12+s11))
        elif self.si < 134.67-s21:
            self.k = -(1/9.125)
        elif self.si < 134.67+s22:
            self.k = 2*(1/9.125)*(self.si-134.67-0.5*(s22-s21))/(s21+s22)
        elif self.si < 249.34-f:
            self.k = (1/9.125)
        else:
            self.k = 0
        return self.k
