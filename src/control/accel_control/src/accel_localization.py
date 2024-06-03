#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs import point_cloud2
import itertools
from sensor_msgs.msg import PointCloud2


class AccelLocalization():

    def __init__(self):
        self.ransac_threshold = 0.5

    def get_route(self, msg: PointCloud2):
        cones_all = point_cloud2.read_points(msg, field_names=("x", "y", "z","color", "score"), skip_nans=True)
        cones = [[c[0],c[1]] for c in cones_all]
        if len(cones)<3:
            return [0,0]
        conos_parejas = itertools.combinations(cones,2)
        conos_trios = [[c1,c2,c3] for c3 in cones for c1,c2 in conos_parejas if c3 not in [c1,c2]]
        #rospy.loginfo(cones)

        max_inliers = 0
        best_coef = [0,0]
        # generators=None
        for c1,c2,c3 in conos_trios:
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
