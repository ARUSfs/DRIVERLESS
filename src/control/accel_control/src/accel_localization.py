#!/usr/bin/env python3

import random
import rospy
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class AccelLocalization():

    def __init__(self):
        self.max_iterations = 50
        self.ransac_threshold = 0.5

    def get_route(self, msg: PointCloud2):
        
        cones_all = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
        cones = [[c[0],c[1]] for c in cones_all]
        #rospy.loginfo(cones)

        max_inliers = 0
        best_coef = [0,0]
        # generators=None
        for _ in range(self.max_iterations):
            i,j,k = random.choices(range(len(cones)),k=3)
            c1=cones[i]
            c2=cones[j]
            c3=cones[k]
            
            #coeficientes de la recta que pasa por c1 y c2 (y=ax+b)
            if c1[0]==c2[0]:
                continue
            else:
                a = (c2[1]-c1[1])/(c2[0]-c1[0])
            b = c1[1] - c1[0]*a
            #coeficiente de la recta paralela a la anterior que pasa por c3
            b2 = c3[1]-a*c3[0]

            inliers=0
            for c in cones:
                #la distacia es la menor entre la distancia a ambas rectas
                d1 = np.abs(a*c[0] + b - c[1])/np.sqrt(a**2 + 1)
                d2 = np.abs(a*c[0] + b2 - c[1])/np.sqrt(a**2 + 1)
                if min(d1,d2)<self.ransac_threshold:
                    inliers+=1

            if inliers>max_inliers:
                best_coef = [a,(b+b2)/2] # coeficientes de la ecuación de la ruta 
                max_inliers=inliers
                # generators = (c1,c2,c3)

        rospy.loginfo(max_inliers)
        rospy.loginfo(best_coef)
        # rospy.loginfo(generators)

        return best_coef
    

        
