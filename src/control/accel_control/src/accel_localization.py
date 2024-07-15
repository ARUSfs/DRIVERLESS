#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs import point_cloud2
import itertools
from sensor_msgs.msg import PointCloud2
import random
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sensor_msgs


global_frame = rospy.get_param('/accel_control/global_frame')
car_frame = rospy.get_param('/accel_control/car_frame')
slam = rospy.get_param('/accel_control/slam')

class AccelLocalization():

    def __init__(self):
        self.ransac_threshold = 0.2
        self.max_iterations = 500
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def get_route(self, msg: PointCloud2):
        if(slam!="none"):
            trans=self.tf_buffer.lookup_transform(car_frame, global_frame, rospy.Time(0))
            trans_msg = tf2_sensor_msgs.do_transform_cloud(msg,transform=trans)
            cones_all = point_cloud2.read_points(trans_msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
            cones = [[c[0],c[1]] for c in cones_all if c[0]>-5]
        else:
            cones_all = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
            cones = [[c[0],c[1]] for c in cones_all]


        if len(cones)<2:
            raise ValueError("1 cono")
        elif len(cones)==2:
            raise ValueError("2 conos")
            
        max_inliers = 0
        best_coef = None
        
        for _ in range(self.max_iterations):
            i,j,k = random.choices(range(len(cones)),k=3)
            c1=cones[i]
            c2=cones[j]
            c3=cones[k]

            if abs(c1[0]-c2[0])<0.1: # checkea que los conos estén en x distintas
                continue

            if abs(c1[1]-c3[1])<0.1 and abs(c2[1]-c3[1])<0.1: # checkea que los conos no sean colineales
                continue
            else:
                a = (c2[1]-c1[1])/(c2[0]-c1[0])
            b = c1[1] - c1[0]*a
            b2 = c3[1]-a*c3[0]

            inliers=0
            for c in cones:
                #la distacia es la menor entre la distancia a ambas rectas
                d1 = np.abs(a*c[0] + b - c[1])/np.sqrt(a**2 + 1)
                d2 = np.abs(a*c[0] + b2 - c[1])/np.sqrt(a**2 + 1)
                if min(d1,d2)<self.ransac_threshold:
                    inliers+=1

            if inliers>max_inliers and abs(a)<np.tan(np.pi/6):
                best_coef = [a,(b+b2)/2] # coeficientes de la ecuación de la ruta 
                max_inliers=inliers
        # rospy.loginfo(max_inliers)
        # rospy.loginfo(best_coef)
        return best_coef 
