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

STANLEY_COEF = rospy.get_param('/skidpad_control/stanley_coef')
K_DELTA = rospy.get_param('/skidpad_control/k_delta')
K_YAW_RATE = rospy.get_param('/skidpad_control/k_yaw_rate')

TARGETILLO = rospy.get_param('/skidpad_control/targetillo')
TARGETASO = rospy.get_param('/skidpad_control/targetaso')
KP = rospy.get_param('/skidpad_control/KP')
KI = rospy.get_param('/skidpad_control/KI')
KD = rospy.get_param('/skidpad_control/KD')


class SkidpadControl():

    def __init__(self):
        
        self.skidpad_localizator = SkidpadLocalization()

        ### Inicializaciones ###

        self.steer = 0
        self.acc = 0
        self.speed = 0
        self.pos = np.array([0,0])
        self.yaw = 0
        self.r = 0

        self.braking = False
        self.integral = 0
        self.prev_err = 0
        self.prev_t = time.time()
        self.steer_diff = 0
        self.prev_steer = 0
        self.delta = 0

        self.centers = []
        self.start_time = None
        self.calib_time = 5
        self.calibrated = False

        self.i = 0
        self.si = 0

 
        self.N = 100
        r=9.125
        d = 2*math.pi*r/self.N

        self.plantilla = np.array([[-20 + d*i,0] for i in range(int(20/d)+1)]+2*[[r * np.sin(2 * np.pi * i / self.N), -9.125+r * np.cos(2 * np.pi * i / self.N)] for i in range(self.N)]+2*[[r * np.sin(2 * np.pi * i / self.N), 9.125-r * np.cos(2 * np.pi * i / self.N)] for i in range(self.N)]+[[d*i,0] for i in range(int(20/d)+1)])
        self.route = None
        
    

        self.cmd_publisher = rospy.Publisher('/controls_pp', Controls, queue_size=1) 
        self.braking_publisher = rospy.Publisher('/braking', Bool, queue_size=10)
        self.route_pub = rospy.Publisher('/skidpad_route',Marker,queue_size=1)
        self.phi_dist_pub = rospy.Publisher('/phi_dist', Float32MultiArray,queue_size=1)
        self.target_speed_pub = rospy.Publisher('/target_speed', Float32, queue_size=10)
        rospy.Subscriber('/perception_map', PointCloud2, self.update_route, queue_size=10)
        rospy.Subscriber('/car_state/state', CarState, self.update_state, queue_size=1)
        rospy.Subscriber('/steering/epos_info', Float32MultiArray, self.update_steer, queue_size=1)



    def update_state(self, msg: CarState):
        self.speed = math.hypot(msg.vx,msg.vy)
        self.pos = np.array([msg.x,msg.y])
        self.yaw = msg.yaw
        self.r = msg.r

        if self.calibrated:
            self.acc = self.get_acc()
            self.steer = self.get_steer()
            self.publish_cmd()
    
    def update_steer(self, msg: Float32MultiArray):
        self.steer_diff = self.prev_steer - msg.data[1]
        self.prev_steer = msg.data[1]


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
            min_samples = int(3*self.calib_time)  # Número mínimo de puntos para formar un clúster

            dbscan = DBSCAN(eps=epsilon, min_samples=min_samples)
            dbscan.fit(np.array(self.centers))

            centroid1_mask = dbscan.labels_ == 0
            centroid2_mask = dbscan.labels_ == 1

            centroid1 = np.mean(np.array(self.centers)[centroid1_mask],axis=0)
            centroid2 = np.mean(np.array(self.centers)[centroid2_mask],axis=0)

            if np.abs(18.25 - np.linalg.norm(centroid1-centroid2)) < 2:
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




    def get_acc(self):
        error = self.get_target() - self.speed

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


    def get_steer(self):
        
        rot = np.array([[math.cos(-self.yaw),-math.sin(-self.yaw)],[math.sin(-self.yaw),math.cos(-self.yaw)]])
        local_route = (self.route-self.pos) @ rot.T

        if self.i+self.N/2 < len(local_route):
            # se halla el punto más cercano al eje delantero
            dist = np.linalg.norm(local_route[self.i:self.i+int(self.N/2),:]-np.array([0.94,0]),axis=1)
            self.i+=np.argmin(dist)
        elif self.i< len(local_route)-1:
            dist = np.linalg.norm(local_route[self.i:,:]-np.array([0.94,0]),axis=1)
            self.i+=np.argmin(dist)
        signo_d = -np.sign(local_route[self.i,1])
        
        d_min=np.min(dist)*signo_d
        dist = d_min+0.0001

        phi = -np.arctan2(local_route[self.i+1,1]-local_route[self.i,1],local_route[self.i+1,0]-local_route[self.i,0])

        d = 2*math.pi*9.125/self.N

        
        # brake 7m after last lap
        if self.i > 4*self.N+int(27/d):
            self.braking=True
            braking_msg = Bool()
            braking_msg.data = True
            self.braking_publisher.publish(braking_msg)
            return self.steer

        self.si = self.i*d
        self.update_k()

        params = Float32MultiArray()
        params.data.append(phi)
        params.data.append(dist)
        self.phi_dist_pub.publish(params)

        r_target = self.speed*self.k

        phi_ss = 250*self.speed*r_target/(-20000*(1+0.55/0.45))

        ### STANLEY CONTROL ###
        delta = -phi -phi_ss - np.arctan(STANLEY_COEF*dist/self.get_target())
        delta += K_DELTA*self.steer_diff + K_YAW_RATE*(r_target - self.r)  

        
        delta = math.degrees(delta)
        self.delta = 0.8*self.delta + 0.2*delta
     
        return max(-20,min(20,self.delta))
    
    def update_k(self):
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
    
    def get_target(self):
        target_speed = 0
        if self.si < 50:
            target_speed = TARGETILLO
        elif self.si < 130:
            target_speed = TARGETASO
        elif self.si < 170:
            target_speed = TARGETILLO
        elif self.si < 245:
            target_speed = TARGETASO
        else:
            target_speed = TARGETILLO
        
        if(not self.braking):
            target_msg = Float32()
            target_msg.data = target_speed
            self.target_speed_pub.publish(target_msg)

        return target_speed